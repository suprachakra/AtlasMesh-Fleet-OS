package repository

import (
	context "context"
	"database/sql"
	"encoding/json"
	"errors"
	"fmt"
	"time"

	_ "github.com/jackc/pgx/v5/stdlib"

	"github.com/atlasmesh/fleet-os/services/policy-engine/internal/models"
)

// PolicyRepository defines the persistence interface for policies.
//
//go:generate mockery --name PolicyRepository --output ../mocks --outpkg mocks
//
// All repository methods accept a context for cancellation/tracing, return
// typed errors, and are version-aware to support policy versioning.
type PolicyRepository interface {
	Create(ctx context.Context, policy *models.Policy) error
	GetByID(ctx context.Context, policyID string) (*models.Policy, error)
	Update(ctx context.Context, policy *models.Policy) error
	List(ctx context.Context, filter PolicyFilter) ([]*models.Policy, error)
	Archive(ctx context.Context, policyID string, reason string) error
	HealthCheck(ctx context.Context) error
	Close() error
}

// AuditRepository persists policy and evaluation audit trails.
type AuditRepository interface {
	Create(ctx context.Context, entry *models.AuditEntry) error
	List(ctx context.Context, filter AuditFilter) ([]*models.AuditEntry, error)
	Purge(ctx context.Context, olderThan time.Time) (int64, error)
	Close() error
}

// PolicyFilter encapsulates optional query parameters used for listing.
type PolicyFilter struct {
	Scope    models.PolicyScope
	Status   models.PolicyStatus
	Type     models.PolicyType
	TenantID string
	Sector   string
	Tags     []string
	IsLatest *bool
	Limit    int
	Offset   int
}

// AuditFilter mirrors PolicyFilter for audit queries.
type AuditFilter struct {
	Type     models.AuditType
	TenantID string
	UserID   string
	Action   string
	From     time.Time
	To       time.Time
	Limit    int
	Offset   int
}

var (
	// ErrPolicyNotFound indicates no policy matched the query.
	ErrPolicyNotFound = errors.New("policy not found")
	// ErrPolicyVersionDiff indicates optimistic locking failure.
	ErrPolicyVersionDiff = errors.New("policy version conflict")
)

type postgresPolicyRepository struct {
	db *sql.DB
}

// NewPolicyRepository constructs a PostgreSQL-backed repository.
func NewPolicyRepository(dsn string) (PolicyRepository, error) {
	db, err := sql.Open("pgx", dsn)
	if err != nil {
		return nil, fmt.Errorf("failed to open policy database: %w", err)
	}

	if err := db.Ping(); err != nil {
		db.Close()
		return nil, fmt.Errorf("failed to ping policy database: %w", err)
	}

	return &postgresPolicyRepository{db: db}, nil
}

func (r *postgresPolicyRepository) Close() error {
	return r.db.Close()
}

func (r *postgresPolicyRepository) HealthCheck(ctx context.Context) error {
	return r.db.PingContext(ctx)
}

func (r *postgresPolicyRepository) Create(ctx context.Context, policy *models.Policy) error {
	query := `
		INSERT INTO policy_engine.policies (
			policy_id, name, description, version, status, policy_type, policy_content,
			rego_code, scope, scope_ids, conditions, priority, created_by, updated_by,
			validation_rules, test_cases, compliance_frameworks, approval_required,
			metadata, tags
		) VALUES (
			$1, $2, $3, $4, $5, $6, $7,
			$8, $9, $10, $11, $12, $13, $14,
			$15, $16, $17, $18, $19, $20
		)
	`

	_, err := r.db.ExecContext(
		ctx, query,
		policy.ID,
		policy.Name,
		policy.Description,
		policy.Version,
		policy.Status.String(),
		policy.Type.String(),
		marshalJSON(policy.Content),
		nil,
		policy.Scope.String(),
		stringSlice(policy.Metadata["scope_ids"]),
		marshalJSON(policy.Metadata["conditions"]),
		policy.Priority,
		policy.CreatedBy,
		policy.UpdatedBy,
		marshalJSON(policy.Metadata["validation_rules"]),
		marshalJSON(policy.Metadata["test_cases"]),
		stringSlice(policy.Metadata["compliance_frameworks"]),
		false,
		marshalJSON(policy.Metadata),
		policy.Tags,
	)

	return err
}

func (r *postgresPolicyRepository) GetByID(ctx context.Context, policyID string) (*models.Policy, error) {
	const query = `
		SELECT policy_id, name, description, policy_type, policy_content,
			status, version, scope, tags, metadata, created_by, created_at,
			updated_by, updated_at
		FROM policy_engine.policies
		WHERE policy_id = $1 AND is_latest_version = true
	`

	row := r.db.QueryRowContext(ctx, query, policyID)

	var (
		policy       models.Policy
		contentBytes []byte
		tagArray     []string
		metadataJSON []byte
		status       string
		typeStr      string
		scopeStr     string
	)

	if err := row.Scan(
		&policy.ID,
		&policy.Name,
		&policy.Description,
		&typeStr,
		&contentBytes,
		&status,
		&policy.Version,
		&scopeStr,
		&tagArray,
		&metadataJSON,
		&policy.CreatedBy,
		&policy.CreatedAt,
		&policy.UpdatedBy,
		&policy.UpdatedAt,
	); err != nil {
		if errors.Is(err, sql.ErrNoRows) {
			return nil, ErrPolicyNotFound
		}
		return nil, fmt.Errorf("failed to fetch policy: %w", err)
	}

	policy.Type = models.PolicyTypeFromString(typeStr)
	policy.Status = models.PolicyStatusFromString(status)
	policy.Scope = models.PolicyScopeFromString(scopeStr)
	policy.Tags = tagArray

	handleJSON(contentBytes, &policy.Content)
	handleJSON(metadataJSON, &policy.Metadata)

	return &policy, nil
}

func (r *postgresPolicyRepository) Update(ctx context.Context, policy *models.Policy) error {
	query := `
		UPDATE policy_engine.policies
		SET name = $1,
			description = $2,
			policy_content = $3,
			status = $4,
			version = version + 1,
			scope = $5,
			tags = $6,
			metadata = $7,
			updated_by = $8,
			updated_at = NOW()
		WHERE policy_id = $9
		AND version = $10
	`

	result, err := r.db.ExecContext(
		ctx, query,
		policy.Name,
		policy.Description,
		marshalJSON(policy.Content),
		policy.Status.String(),
		policy.Scope.String(),
		policy.Tags,
		marshalJSON(policy.Metadata),
		policy.UpdatedBy,
		policy.ID,
		policy.Version,
	)
	if err != nil {
		return fmt.Errorf("failed to update policy: %w", err)
	}

	affected, err := result.RowsAffected()
	if err != nil {
		return fmt.Errorf("failed to determine affected rows: %w", err)
	}
	if affected == 0 {
		return ErrPolicyVersionDiff
	}

	return nil
}

func (r *postgresPolicyRepository) List(ctx context.Context, filter PolicyFilter) ([]*models.Policy, error) {
	query := `
		SELECT policy_id, name, description, policy_type, policy_content,
			status, version, scope, tags, metadata, created_at, updated_at
		FROM policy_engine.policies
		WHERE 1=1
	`

	args := []interface{}{}
	arg := 1

	if filter.Status != models.PolicyStatusUnspecified {
		query += fmt.Sprintf(" AND status = $%d", arg)
		args = append(args, filter.Status.String())
		arg++
	}
	if filter.Type != models.PolicyTypeUnspecified {
		query += fmt.Sprintf(" AND policy_type = $%d", arg)
		args = append(args, filter.Type.String())
		arg++
	}
	if filter.Scope != models.PolicyScopeUnspecified {
		query += fmt.Sprintf(" AND scope = $%d", arg)
		args = append(args, filter.Scope.String())
		arg++
	}
	if filter.IsLatest != nil {
		query += fmt.Sprintf(" AND is_latest_version = $%d", arg)
		args = append(args, *filter.IsLatest)
		arg++
	}
	if filter.Limit > 0 {
		query += fmt.Sprintf(" LIMIT $%d", arg)
		args = append(args, filter.Limit)
		arg++
	}
	if filter.Offset > 0 {
		query += fmt.Sprintf(" OFFSET $%d", arg)
		args = append(args, filter.Offset)
		arg++
	}

	rows, err := r.db.QueryContext(ctx, query, args...)
	if err != nil {
		return nil, fmt.Errorf("failed to list policies: %w", err)
	}
	defer rows.Close()

	var policies []*models.Policy
	for rows.Next() {
		var p models.Policy
		var contentBytes []byte
		var metadataBytes []byte
		var status, typeStr, scopeStr string
		var tags []string

		if err := rows.Scan(
			&p.ID,
			&p.Name,
			&p.Description,
			&typeStr,
			&contentBytes,
			&status,
			&p.Version,
			&scopeStr,
			&tags,
			&metadataBytes,
			&p.CreatedAt,
			&p.UpdatedAt,
		); err != nil {
			return nil, err
		}

		p.Type = models.PolicyTypeFromString(typeStr)
		p.Status = models.PolicyStatusFromString(status)
		p.Scope = models.PolicyScopeFromString(scopeStr)
		p.Tags = tags

		handleJSON(contentBytes, &p.Content)
		handleJSON(metadataBytes, &p.Metadata)

		policies = append(policies, &p)
	}

	return policies, rows.Err()
}

func (r *postgresPolicyRepository) Archive(ctx context.Context, policyID string, reason string) error {
	query := `
		UPDATE policy_engine.policies
		SET status = 'archived', metadata = metadata || jsonb_build_object('archive_reason', $1)
		WHERE policy_id = $2
	`

	result, err := r.db.ExecContext(ctx, query, reason, policyID)
	if err != nil {
		return fmt.Errorf("failed to archive policy: %w", err)
	}

	affected, err := result.RowsAffected()
	if err != nil {
		return err
	}
	if affected == 0 {
		return ErrPolicyNotFound
	}

	return nil
}

type postgresAuditRepository struct {
	db *sql.DB
}

// NewAuditRepository constructs a PostgreSQL-backed audit repository.
func NewAuditRepository(dsn string) (AuditRepository, error) {
	db, err := sql.Open("pgx", dsn)
	if err != nil {
		return nil, fmt.Errorf("failed to open audit database: %w", err)
	}
	if err := db.Ping(); err != nil {
		db.Close()
		return nil, fmt.Errorf("failed to ping audit database: %w", err)
	}

	return &postgresAuditRepository{db: db}, nil
}

func (r *postgresAuditRepository) Close() error {
	return r.db.Close()
}

func (r *postgresAuditRepository) Create(ctx context.Context, entry *models.AuditEntry) error {
	query := `
		INSERT INTO audit.policy_audit_log (
			audit_id, policy_id, action, old_values, new_values, changed_fields,
			changed_by, changed_at, metadata
		) VALUES ($1,$2,$3,$4,$5,$6,$7,$8,$9)
	`

	_, err := r.db.ExecContext(
		ctx, query,
		entry.ID,
		entry.Resource,
		entry.Action,
		marshalJSON(entry.Metadata["old_values"]),
		marshalJSON(entry.Metadata["new_values"]),
		stringSlice(entry.Metadata["changed_fields"]),
		entry.UserID,
		entry.Timestamp,
		marshalJSON(entry.Metadata),
	)
	return err
}

func (r *postgresAuditRepository) List(ctx context.Context, filter AuditFilter) ([]*models.AuditEntry, error) {
	query := `
		SELECT audit_id, policy_id, action, metadata, changed_by, changed_at
		FROM audit.policy_audit_log
		WHERE 1=1
	`

	args := []interface{}{}
	arg := 1

	if filter.Action != "" {
		query += fmt.Sprintf(" AND action = $%d", arg)
		args = append(args, filter.Action)
		arg++
	}
	if !filter.From.IsZero() {
		query += fmt.Sprintf(" AND changed_at >= $%d", arg)
		args = append(args, filter.From)
		arg++
	}
	if !filter.To.IsZero() {
		query += fmt.Sprintf(" AND changed_at <= $%d", arg)
		args = append(args, filter.To)
		arg++
	}

	rows, err := r.db.QueryContext(ctx, query, args...)
	if err != nil {
		return nil, fmt.Errorf("failed to list audit entries: %w", err)
	}
	defer rows.Close()

	var entries []*models.AuditEntry
	for rows.Next() {
		var entry models.AuditEntry
		var metadataJSON []byte

		if err := rows.Scan(
			&entry.ID,
			&entry.Resource,
			&entry.Action,
			&metadataJSON,
			&entry.UserID,
			&entry.Timestamp,
		); err != nil {
			return nil, err
		}

		handleJSON(metadataJSON, &entry.Metadata)
		entries = append(entries, &entry)
	}

	return entries, rows.Err()
}

func (r *postgresAuditRepository) Purge(ctx context.Context, olderThan time.Time) (int64, error) {
	result, err := r.db.ExecContext(ctx, "DELETE FROM audit.policy_audit_log WHERE changed_at < $1", olderThan)
	if err != nil {
		return 0, fmt.Errorf("failed to purge audit log: %w", err)
	}
	return result.RowsAffected()
}

// --- helpers ---

func marshalJSON(value interface{}) []byte {
	if value == nil {
		return []byte("{}")
	}
	b, err := json.Marshal(value)
	if err != nil {
		return []byte("{}")
	}
	return b
}

func handleJSON(data []byte, target interface{}) {
	if len(data) == 0 {
		return
	}
	_ = json.Unmarshal(data, target)
}

func stringSlice(value interface{}) []string {
	switch v := value.(type) {
	case []string:
		return v
	case []interface{}:
		out := make([]string, 0, len(v))
		for _, item := range v {
			if str, ok := item.(string); ok {
				out = append(out, str)
			}
		}
		return out
	default:
		return []string{}
	}
}
