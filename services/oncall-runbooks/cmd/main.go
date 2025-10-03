package main

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/signal"
	"sync"
	"syscall"
	"time"

	"github.com/gorilla/mux"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"go.opentelemetry.io/otel"
	"go.opentelemetry.io/otel/attribute"
	"go.opentelemetry.io/otel/trace"
)

// OnCallRunbooksService manages incident response, runbooks, and on-call operations
type OnCallRunbooksService struct {
	incidentManager    *IncidentManager
	runbookManager     *RunbookManager
	pagingManager      *PagingManager
	alertManager       *AlertManager
	escalationManager  *EscalationManager
	fatiguePreventor   *FatiguePreventor
	metricsCollector   *MetricsCollector
	metrics            *OnCallMetrics
	tracer             trace.Tracer
	config             *Config
}

// Config holds the service configuration
type Config struct {
	Port                    int                     `json:"port"`
	MetricsPort             int                     `json:"metrics_port"`
	IncidentConfig          IncidentConfig          `json:"incident"`
	RunbookConfig           RunbookConfig           `json:"runbook"`
	PagingConfig            PagingConfig            `json:"paging"`
	AlertConfig             AlertConfig             `json:"alert"`
	EscalationConfig        EscalationConfig        `json:"escalation"`
	FatiguePreventionConfig FatiguePreventionConfig `json:"fatigue_prevention"`
	NotificationConfig      NotificationConfig      `json:"notification"`
	SchedulingConfig        SchedulingConfig        `json:"scheduling"`
}

// IncidentConfig configures incident management
type IncidentConfig struct {
	AutoClassification      bool                    `json:"auto_classification"`
	SeverityLevels          []SeverityLevel         `json:"severity_levels"`
	IncidentTypes          []IncidentType          `json:"incident_types"`
	ResponseTimeTargets    map[string]time.Duration `json:"response_time_targets"`
	ResolutionTimeTargets  map[string]time.Duration `json:"resolution_time_targets"`
	AutoAssignment         bool                    `json:"auto_assignment"`
	StatusUpdateInterval   time.Duration           `json:"status_update_interval"`
	PostMortemRequired     []string                `json:"post_mortem_required"`
}

// RunbookConfig configures runbook management
type RunbookConfig struct {
	Repository             string                  `json:"repository"`
	AutoSuggestion         bool                    `json:"auto_suggestion"`
	VersionControl         bool                    `json:"version_control"`
	ApprovalRequired       bool                    `json:"approval_required"`
	TestingRequired        bool                    `json:"testing_required"`
	UpdateNotifications    bool                    `json:"update_notifications"`
	SearchEnabled          bool                    `json:"search_enabled"`
	TemplateLibrary        []RunbookTemplate       `json:"template_library"`
}

// PagingConfig configures paging and notification rules
type PagingConfig struct {
	PagingEnabled          bool                    `json:"paging_enabled"`
	PagingChannels         []PagingChannel         `json:"paging_channels"`
	PagingRules            []PagingRule            `json:"paging_rules"`
	QuietHours             []QuietHour             `json:"quiet_hours"`
	EscalationDelay        time.Duration           `json:"escalation_delay"`
	MaxRetries             int                     `json:"max_retries"`
	RetryInterval          time.Duration           `json:"retry_interval"`
	AcknowledgmentTimeout  time.Duration           `json:"acknowledgment_timeout"`
}

// AlertConfig configures alert processing and filtering
type AlertConfig struct {
	AlertSources           []AlertSource           `json:"alert_sources"`
	FilteringRules         []FilteringRule         `json:"filtering_rules"`
	GroupingRules          []GroupingRule          `json:"grouping_rules"`
	SuppressionRules       []SuppressionRule       `json:"suppression_rules"`
	ThrottlingRules        []ThrottlingRule        `json:"throttling_rules"`
	DeduplicationEnabled   bool                    `json:"deduplication_enabled"`
	DeduplicationWindow    time.Duration           `json:"deduplication_window"`
}

// EscalationConfig configures escalation policies
type EscalationConfig struct {
	EscalationPolicies     []EscalationPolicy      `json:"escalation_policies"`
	DefaultPolicy          string                  `json:"default_policy"`
	EscalationTimeout      time.Duration           `json:"escalation_timeout"`
	ManagerEscalation      bool                    `json:"manager_escalation"`
	ExecutiveEscalation    bool                    `json:"executive_escalation"`
	ExternalEscalation     bool                    `json:"external_escalation"`
}

// FatiguePreventionConfig configures alert fatigue prevention
type FatiguePreventionConfig struct {
	Enabled                bool                    `json:"enabled"`
	AlertVelocityThreshold int                     `json:"alert_velocity_threshold"`
	AlertVelocityWindow    time.Duration           `json:"alert_velocity_window"`
	SilenceThreshold       int                     `json:"silence_threshold"`
	SilenceDuration        time.Duration           `json:"silence_duration"`
	IntelligentGrouping    bool                    `json:"intelligent_grouping"`
	AdaptiveThresholds     bool                    `json:"adaptive_thresholds"`
	FatigueMetrics         []FatigueMetric         `json:"fatigue_metrics"`
}

// NotificationConfig configures notification channels
type NotificationConfig struct {
	Channels               []NotificationChannel   `json:"channels"`
	Templates              []NotificationTemplate  `json:"templates"`
	RateLimiting           bool                    `json:"rate_limiting"`
	RateLimitRules         []RateLimitRule         `json:"rate_limit_rules"`
	DeliveryTracking       bool                    `json:"delivery_tracking"`
	FailureRetry           bool                    `json:"failure_retry"`
}

// SchedulingConfig configures on-call scheduling
type SchedulingConfig struct {
	ScheduleEnabled        bool                    `json:"schedule_enabled"`
	Schedules              []OnCallSchedule        `json:"schedules"`
	RotationTypes          []RotationType          `json:"rotation_types"`
	HandoffReminders       bool                    `json:"handoff_reminders"`
	CoverageValidation     bool                    `json:"coverage_validation"`
	TimeZoneSupport        bool                    `json:"timezone_support"`
}

// Incident represents an incident
type Incident struct {
	ID                     string                  `json:"id"`
	Title                  string                  `json:"title"`
	Description            string                  `json:"description"`
	Severity               string                  `json:"severity"`
	Priority               string                  `json:"priority"`
	Status                 string                  `json:"status"`
	Type                   string                  `json:"type"`
	Category               string                  `json:"category"`
	Source                 string                  `json:"source"`
	AffectedServices       []string                `json:"affected_services"`
	AssignedTo             string                  `json:"assigned_to"`
	Responders             []Responder             `json:"responders"`
	CreatedAt              time.Time               `json:"created_at"`
	UpdatedAt              time.Time               `json:"updated_at"`
	AcknowledgedAt         *time.Time              `json:"acknowledged_at,omitempty"`
	ResolvedAt             *time.Time              `json:"resolved_at,omitempty"`
	ResponseTime           *time.Duration          `json:"response_time,omitempty"`
	ResolutionTime         *time.Duration          `json:"resolution_time,omitempty"`
	Timeline               []IncidentEvent         `json:"timeline"`
	Runbooks               []string                `json:"runbooks"`
	RelatedIncidents       []string                `json:"related_incidents"`
	PostMortem             *PostMortem             `json:"post_mortem,omitempty"`
	Metrics                IncidentMetrics         `json:"metrics"`
	Metadata               map[string]string       `json:"metadata"`
}

// Runbook represents an operational runbook
type Runbook struct {
	ID                     string                  `json:"id"`
	Name                   string                  `json:"name"`
	Description            string                  `json:"description"`
	Version                string                  `json:"version"`
	Category               string                  `json:"category"`
	Tags                   []string                `json:"tags"`
	Triggers               []RunbookTrigger        `json:"triggers"`
	Steps                  []RunbookStep           `json:"steps"`
	Prerequisites          []string                `json:"prerequisites"`
	EstimatedDuration      time.Duration           `json:"estimated_duration"`
	DifficultyLevel        string                  `json:"difficulty_level"`
	RequiredPermissions    []string                `json:"required_permissions"`
	Author                 string                  `json:"author"`
	Reviewers              []string                `json:"reviewers"`
	ApprovedBy             string                  `json:"approved_by"`
	CreatedAt              time.Time               `json:"created_at"`
	UpdatedAt              time.Time               `json:"updated_at"`
	LastTested             *time.Time              `json:"last_tested,omitempty"`
	TestResults            []TestResult            `json:"test_results"`
	UsageStats             UsageStats              `json:"usage_stats"`
	Feedback               []Feedback              `json:"feedback"`
	Metadata               map[string]string       `json:"metadata"`
}

// Alert represents an alert
type Alert struct {
	ID                     string                  `json:"id"`
	Name                   string                  `json:"name"`
	Description            string                  `json:"description"`
	Severity               string                  `json:"severity"`
	Priority               string                  `json:"priority"`
	Status                 string                  `json:"status"`
	Source                 string                  `json:"source"`
	Labels                 map[string]string       `json:"labels"`
	Annotations            map[string]string       `json:"annotations"`
	StartsAt               time.Time               `json:"starts_at"`
	EndsAt                 *time.Time              `json:"ends_at,omitempty"`
	GeneratorURL           string                  `json:"generator_url"`
	Fingerprint            string                  `json:"fingerprint"`
	GroupKey               string                  `json:"group_key"`
	SilencedBy             []string                `json:"silenced_by"`
	InhibitedBy            []string                `json:"inhibited_by"`
	RelatedIncidents       []string                `json:"related_incidents"`
	SuggestedRunbooks      []string                `json:"suggested_runbooks"`
	ProcessingHistory      []ProcessingEvent       `json:"processing_history"`
	Metadata               map[string]string       `json:"metadata"`
}

// OnCallSchedule represents an on-call schedule
type OnCallSchedule struct {
	ID                     string                  `json:"id"`
	Name                   string                  `json:"name"`
	Description            string                  `json:"description"`
	Team                   string                  `json:"team"`
	TimeZone               string                  `json:"timezone"`
	RotationType           string                  `json:"rotation_type"`
	RotationDuration       time.Duration           `json:"rotation_duration"`
	Participants           []OnCallParticipant     `json:"participants"`
	CurrentOnCall          []string                `json:"current_on_call"`
	NextRotation           time.Time               `json:"next_rotation"`
	EscalationPolicy       string                  `json:"escalation_policy"`
	OverrideRules          []OverrideRule          `json:"override_rules"`
	HandoffChecklist       []string                `json:"handoff_checklist"`
	CreatedAt              time.Time               `json:"created_at"`
	UpdatedAt              time.Time               `json:"updated_at"`
	Metadata               map[string]string       `json:"metadata"`
}

// Supporting types
type SeverityLevel struct {
	Name                   string                  `json:"name"`
	Level                  int                     `json:"level"`
	Description            string                  `json:"description"`
	ResponseTimeTarget     time.Duration           `json:"response_time_target"`
	ResolutionTimeTarget   time.Duration           `json:"resolution_time_target"`
	EscalationPolicy       string                  `json:"escalation_policy"`
	NotificationChannels   []string                `json:"notification_channels"`
}

type IncidentType struct {
	Name                   string                  `json:"name"`
	Category               string                  `json:"category"`
	Description            string                  `json:"description"`
	DefaultSeverity        string                  `json:"default_severity"`
	AutoAssignmentRules    []AssignmentRule        `json:"auto_assignment_rules"`
	SuggestedRunbooks      []string                `json:"suggested_runbooks"`
	RequiredFields         []string                `json:"required_fields"`
}

type Responder struct {
	UserID                 string                  `json:"user_id"`
	Name                   string                  `json:"name"`
	Role                   string                  `json:"role"`
	Team                   string                  `json:"team"`
	JoinedAt               time.Time               `json:"joined_at"`
	Status                 string                  `json:"status"`
	ContactInfo            ContactInfo             `json:"contact_info"`
}

type IncidentEvent struct {
	ID                     string                  `json:"id"`
	Type                   string                  `json:"type"`
	Description            string                  `json:"description"`
	Timestamp              time.Time               `json:"timestamp"`
	Actor                  string                  `json:"actor"`
	Details                map[string]interface{}  `json:"details"`
}

type PostMortem struct {
	ID                     string                  `json:"id"`
	IncidentID             string                  `json:"incident_id"`
	Summary                string                  `json:"summary"`
	Timeline               []PostMortemEvent       `json:"timeline"`
	RootCause              string                  `json:"root_cause"`
	ContributingFactors    []string                `json:"contributing_factors"`
	Impact                 Impact                  `json:"impact"`
	LessonsLearned         []string                `json:"lessons_learned"`
	ActionItems            []ActionItem            `json:"action_items"`
	Author                 string                  `json:"author"`
	Reviewers              []string                `json:"reviewers"`
	CreatedAt              time.Time               `json:"created_at"`
	PublishedAt            *time.Time              `json:"published_at,omitempty"`
}

type IncidentMetrics struct {
	MTTR                   time.Duration           `json:"mttr"`
	MTTA                   time.Duration           `json:"mtta"`
	Downtime               time.Duration           `json:"downtime"`
	AffectedUsers          int                     `json:"affected_users"`
	BusinessImpact         float64                 `json:"business_impact"`
	CustomerComplaints     int                     `json:"customer_complaints"`
}

type RunbookTrigger struct {
	Type                   string                  `json:"type"`
	Condition              string                  `json:"condition"`
	AlertPattern           string                  `json:"alert_pattern,omitempty"`
	IncidentType           string                  `json:"incident_type,omitempty"`
	ServicePattern         string                  `json:"service_pattern,omitempty"`
}

type RunbookStep struct {
	ID                     string                  `json:"id"`
	Name                   string                  `json:"name"`
	Description            string                  `json:"description"`
	Type                   string                  `json:"type"`
	Command                string                  `json:"command,omitempty"`
	Script                 string                  `json:"script,omitempty"`
	CheckCommand           string                  `json:"check_command,omitempty"`
	ExpectedResult         string                  `json:"expected_result,omitempty"`
	Timeout                time.Duration           `json:"timeout"`
	RetryCount             int                     `json:"retry_count"`
	ContinueOnFailure      bool                    `json:"continue_on_failure"`
	RequiresApproval       bool                    `json:"requires_approval"`
	DangerLevel            string                  `json:"danger_level"`
	Prerequisites          []string                `json:"prerequisites"`
	NextSteps              []string                `json:"next_steps"`
}

type TestResult struct {
	ID                     string                  `json:"id"`
	TestDate               time.Time               `json:"test_date"`
	Tester                 string                  `json:"tester"`
	Success                bool                    `json:"success"`
	Duration               time.Duration           `json:"duration"`
	FailedSteps            []string                `json:"failed_steps"`
	Notes                  string                  `json:"notes"`
	Environment            string                  `json:"environment"`
}

type UsageStats struct {
	ExecutionCount         int                     `json:"execution_count"`
	SuccessRate            float64                 `json:"success_rate"`
	AverageExecutionTime   time.Duration           `json:"average_execution_time"`
	LastUsed               time.Time               `json:"last_used"`
	PopularSteps           []string                `json:"popular_steps"`
	CommonFailures         []string                `json:"common_failures"`
}

type Feedback struct {
	ID                     string                  `json:"id"`
	UserID                 string                  `json:"user_id"`
	Rating                 int                     `json:"rating"`
	Comment                string                  `json:"comment"`
	Suggestions            []string                `json:"suggestions"`
	CreatedAt              time.Time               `json:"created_at"`
}

type PagingChannel struct {
	Name                   string                  `json:"name"`
	Type                   string                  `json:"type"`
	Configuration          map[string]interface{}  `json:"configuration"`
	Priority               int                     `json:"priority"`
	Enabled                bool                    `json:"enabled"`
	RateLimits             []RateLimit             `json:"rate_limits"`
}

type PagingRule struct {
	ID                     string                  `json:"id"`
	Name                   string                  `json:"name"`
	Conditions             []Condition             `json:"conditions"`
	Actions                []PagingAction          `json:"actions"`
	Priority               int                     `json:"priority"`
	Enabled                bool                    `json:"enabled"`
	Schedule               *Schedule               `json:"schedule,omitempty"`
}

type QuietHour struct {
	Name                   string                  `json:"name"`
	StartTime              string                  `json:"start_time"`
	EndTime                string                  `json:"end_time"`
	TimeZone               string                  `json:"timezone"`
	DaysOfWeek             []string                `json:"days_of_week"`
	SeverityExceptions     []string                `json:"severity_exceptions"`
	ServiceExceptions      []string                `json:"service_exceptions"`
}

type AlertSource struct {
	Name                   string                  `json:"name"`
	Type                   string                  `json:"type"`
	URL                    string                  `json:"url"`
	Configuration          map[string]interface{}  `json:"configuration"`
	Enabled                bool                    `json:"enabled"`
	ProcessingRules        []ProcessingRule        `json:"processing_rules"`
}

type FilteringRule struct {
	ID                     string                  `json:"id"`
	Name                   string                  `json:"name"`
	Conditions             []Condition             `json:"conditions"`
	Action                 string                  `json:"action"`
	Reason                 string                  `json:"reason"`
	Enabled                bool                    `json:"enabled"`
}

type GroupingRule struct {
	ID                     string                  `json:"id"`
	Name                   string                  `json:"name"`
	GroupBy                []string                `json:"group_by"`
	GroupWait              time.Duration           `json:"group_wait"`
	GroupInterval          time.Duration           `json:"group_interval"`
	RepeatInterval         time.Duration           `json:"repeat_interval"`
	Conditions             []Condition             `json:"conditions"`
}

type SuppressionRule struct {
	ID                     string                  `json:"id"`
	Name                   string                  `json:"name"`
	Conditions             []Condition             `json:"conditions"`
	Duration               time.Duration           `json:"duration"`
	Reason                 string                  `json:"reason"`
	CreatedBy              string                  `json:"created_by"`
	CreatedAt              time.Time               `json:"created_at"`
	ExpiresAt              time.Time               `json:"expires_at"`
}

type ThrottlingRule struct {
	ID                     string                  `json:"id"`
	Name                   string                  `json:"name"`
	Conditions             []Condition             `json:"conditions"`
	MaxAlerts              int                     `json:"max_alerts"`
	TimeWindow             time.Duration           `json:"time_window"`
	Action                 string                  `json:"action"`
}

type EscalationPolicy struct {
	ID                     string                  `json:"id"`
	Name                   string                  `json:"name"`
	Description            string                  `json:"description"`
	Steps                  []EscalationStep        `json:"steps"`
	RepeatCount            int                     `json:"repeat_count"`
	RepeatInterval         time.Duration           `json:"repeat_interval"`
	Conditions             []Condition             `json:"conditions"`
}

type EscalationStep struct {
	StepNumber             int                     `json:"step_number"`
	Delay                  time.Duration           `json:"delay"`
	Targets                []EscalationTarget      `json:"targets"`
	NotificationChannels   []string                `json:"notification_channels"`
	RequireAcknowledgment  bool                    `json:"require_acknowledgment"`
	Timeout                time.Duration           `json:"timeout"`
}

type EscalationTarget struct {
	Type                   string                  `json:"type"`
	ID                     string                  `json:"id"`
	Name                   string                  `json:"name"`
	ContactInfo            ContactInfo             `json:"contact_info"`
}

type FatigueMetric struct {
	Name                   string                  `json:"name"`
	Type                   string                  `json:"type"`
	Threshold              float64                 `json:"threshold"`
	Window                 time.Duration           `json:"window"`
	Action                 string                  `json:"action"`
}

type NotificationChannel struct {
	ID                     string                  `json:"id"`
	Name                   string                  `json:"name"`
	Type                   string                  `json:"type"`
	Configuration          map[string]interface{}  `json:"configuration"`
	Enabled                bool                    `json:"enabled"`
	DeliverySettings       DeliverySettings        `json:"delivery_settings"`
}

type NotificationTemplate struct {
	ID                     string                  `json:"id"`
	Name                   string                  `json:"name"`
	Type                   string                  `json:"type"`
	Subject                string                  `json:"subject"`
	Body                   string                  `json:"body"`
	Variables              []string                `json:"variables"`
	Conditions             []Condition             `json:"conditions"`
}

type RateLimitRule struct {
	Channel                string                  `json:"channel"`
	MaxMessages            int                     `json:"max_messages"`
	TimeWindow             time.Duration           `json:"time_window"`
	BurstAllowance         int                     `json:"burst_allowance"`
}

type OnCallParticipant struct {
	UserID                 string                  `json:"user_id"`
	Name                   string                  `json:"name"`
	Role                   string                  `json:"role"`
	Team                   string                  `json:"team"`
	ContactInfo            ContactInfo             `json:"contact_info"`
	Availability           []AvailabilityWindow    `json:"availability"`
	Skills                 []string                `json:"skills"`
	PreferredShifts        []string                `json:"preferred_shifts"`
}

type RotationType struct {
	Name                   string                  `json:"name"`
	Duration               time.Duration           `json:"duration"`
	HandoffTime            string                  `json:"handoff_time"`
	TimeZone               string                  `json:"timezone"`
	WeekendHandling        string                  `json:"weekend_handling"`
	HolidayHandling        string                  `json:"holiday_handling"`
}

type OverrideRule struct {
	ID                     string                  `json:"id"`
	Name                   string                  `json:"name"`
	StartTime              time.Time               `json:"start_time"`
	EndTime                time.Time               `json:"end_time"`
	OverrideUser           string                  `json:"override_user"`
	Reason                 string                  `json:"reason"`
	CreatedBy              string                  `json:"created_by"`
	ApprovedBy             string                  `json:"approved_by,omitempty"`
}

type ContactInfo struct {
	Email                  string                  `json:"email"`
	Phone                  string                  `json:"phone"`
	SMS                    string                  `json:"sms"`
	Slack                  string                  `json:"slack"`
	Teams                  string                  `json:"teams"`
	PagerDuty              string                  `json:"pagerduty"`
}

type DeliverySettings struct {
	RetryCount             int                     `json:"retry_count"`
	RetryInterval          time.Duration           `json:"retry_interval"`
	Timeout                time.Duration           `json:"timeout"`
	FailureEscalation      bool                    `json:"failure_escalation"`
}

type AvailabilityWindow struct {
	DayOfWeek              string                  `json:"day_of_week"`
	StartTime              string                  `json:"start_time"`
	EndTime                string                  `json:"end_time"`
	TimeZone               string                  `json:"timezone"`
}

type Condition struct {
	Field                  string                  `json:"field"`
	Operator               string                  `json:"operator"`
	Value                  interface{}             `json:"value"`
	LogicalOperator        string                  `json:"logical_operator,omitempty"`
}

type PagingAction struct {
	Type                   string                  `json:"type"`
	Target                 string                  `json:"target"`
	Channel                string                  `json:"channel"`
	Template               string                  `json:"template"`
	Delay                  time.Duration           `json:"delay"`
	Parameters             map[string]interface{}  `json:"parameters"`
}

type Schedule struct {
	StartTime              string                  `json:"start_time"`
	EndTime                string                  `json:"end_time"`
	TimeZone               string                  `json:"timezone"`
	DaysOfWeek             []string                `json:"days_of_week"`
	Holidays               []string                `json:"holidays"`
}

type ProcessingRule struct {
	Name                   string                  `json:"name"`
	Type                   string                  `json:"type"`
	Conditions             []Condition             `json:"conditions"`
	Transformations        []Transformation        `json:"transformations"`
	Enabled                bool                    `json:"enabled"`
}

type Transformation struct {
	Field                  string                  `json:"field"`
	Operation              string                  `json:"operation"`
	Value                  interface{}             `json:"value"`
}

type ProcessingEvent struct {
	Timestamp              time.Time               `json:"timestamp"`
	Action                 string                  `json:"action"`
	Component              string                  `json:"component"`
	Details                map[string]interface{}  `json:"details"`
}

type AssignmentRule struct {
	Conditions             []Condition             `json:"conditions"`
	AssignTo               string                  `json:"assign_to"`
	Team                   string                  `json:"team"`
	Skill                  string                  `json:"skill"`
	Priority               int                     `json:"priority"`
}

type PostMortemEvent struct {
	Timestamp              time.Time               `json:"timestamp"`
	Description            string                  `json:"description"`
	Type                   string                  `json:"type"`
	Impact                 string                  `json:"impact"`
}

type Impact struct {
	Duration               time.Duration           `json:"duration"`
	AffectedUsers          int                     `json:"affected_users"`
	AffectedServices       []string                `json:"affected_services"`
	BusinessImpact         string                  `json:"business_impact"`
	FinancialImpact        float64                 `json:"financial_impact"`
	ReputationImpact       string                  `json:"reputation_impact"`
}

type ActionItem struct {
	ID                     string                  `json:"id"`
	Description            string                  `json:"description"`
	Owner                  string                  `json:"owner"`
	Priority               string                  `json:"priority"`
	DueDate                time.Time               `json:"due_date"`
	Status                 string                  `json:"status"`
	CreatedAt              time.Time               `json:"created_at"`
	CompletedAt            *time.Time              `json:"completed_at,omitempty"`
}

type RateLimit struct {
	MaxMessages            int                     `json:"max_messages"`
	TimeWindow             time.Duration           `json:"time_window"`
	BurstAllowance         int                     `json:"burst_allowance"`
}

type RunbookTemplate struct {
	ID                     string                  `json:"id"`
	Name                   string                  `json:"name"`
	Category               string                  `json:"category"`
	Description            string                  `json:"description"`
	Template               string                  `json:"template"`
	Variables              []string                `json:"variables"`
	DefaultSteps           []RunbookStep           `json:"default_steps"`
}

// Service components
type IncidentManager struct {
	config      *IncidentConfig
	incidents   map[string]*Incident
	classifier  IncidentClassifier
	assigner    IncidentAssigner
	metrics     *OnCallMetrics
	mu          sync.RWMutex
}

type RunbookManager struct {
	config      *RunbookConfig
	runbooks    map[string]*Runbook
	repository  RunbookRepository
	suggester   RunbookSuggester
	executor    RunbookExecutor
	metrics     *OnCallMetrics
	mu          sync.RWMutex
}

type PagingManager struct {
	config      *PagingConfig
	rules       []PagingRule
	channels    map[string]PagingChannel
	scheduler   PagingScheduler
	deliverer   NotificationDeliverer
	metrics     *OnCallMetrics
	mu          sync.RWMutex
}

type AlertManager struct {
	config      *AlertConfig
	alerts      map[string]*Alert
	processor   AlertProcessor
	grouper     AlertGrouper
	suppressor  AlertSuppressor
	metrics     *OnCallMetrics
	mu          sync.RWMutex
}

type EscalationManager struct {
	config      *EscalationConfig
	policies    map[string]*EscalationPolicy
	executor    EscalationExecutor
	tracker     EscalationTracker
	metrics     *OnCallMetrics
	mu          sync.RWMutex
}

type FatiguePreventor struct {
	config      *FatiguePreventionConfig
	analyzer    FatigueAnalyzer
	mitigator   FatigueMitigator
	metrics     *OnCallMetrics
	mu          sync.RWMutex
}

type MetricsCollector struct {
	metrics     *OnCallMetrics
	collectors  map[string]MetricCollector
	mu          sync.RWMutex
}

// OnCallMetrics contains Prometheus metrics
type OnCallMetrics struct {
	IncidentsTotal          *prometheus.CounterVec
	IncidentDuration        *prometheus.HistogramVec
	AlertsTotal             *prometheus.CounterVec
	AlertProcessingTime     *prometheus.HistogramVec
	PagingDeliveryTime      *prometheus.HistogramVec
	EscalationEvents        *prometheus.CounterVec
	RunbookExecutions       *prometheus.CounterVec
	FatigueScore            *prometheus.GaugeVec
	ResponseTime            *prometheus.HistogramVec
	ResolutionTime          *prometheus.HistogramVec
	OnCallCoverage          *prometheus.GaugeVec
}

func main() {
	// Load configuration
	config := loadConfig()
	
	// Initialize metrics
	metrics := initializeMetrics()
	
	// Initialize tracer
	tracer := otel.Tracer("oncall-runbooks-service")
	
	// Initialize incident manager
	incidentManager := &IncidentManager{
		config:     &config.IncidentConfig,
		incidents:  make(map[string]*Incident),
		classifier: NewIncidentClassifier(config.IncidentConfig),
		assigner:   NewIncidentAssigner(config.IncidentConfig),
		metrics:    metrics,
	}
	
	// Initialize runbook manager
	runbookManager := &RunbookManager{
		config:     &config.RunbookConfig,
		runbooks:   make(map[string]*Runbook),
		repository: NewRunbookRepository(config.RunbookConfig),
		suggester:  NewRunbookSuggester(config.RunbookConfig),
		executor:   NewRunbookExecutor(config.RunbookConfig),
		metrics:    metrics,
	}
	
	// Initialize paging manager
	pagingManager := &PagingManager{
		config:    &config.PagingConfig,
		rules:     config.PagingConfig.PagingRules,
		channels:  make(map[string]PagingChannel),
		scheduler: NewPagingScheduler(config.PagingConfig),
		deliverer: NewNotificationDeliverer(config.NotificationConfig),
		metrics:   metrics,
	}
	
	// Initialize alert manager
	alertManager := &AlertManager{
		config:     &config.AlertConfig,
		alerts:     make(map[string]*Alert),
		processor:  NewAlertProcessor(config.AlertConfig),
		grouper:    NewAlertGrouper(config.AlertConfig),
		suppressor: NewAlertSuppressor(config.AlertConfig),
		metrics:    metrics,
	}
	
	// Initialize escalation manager
	escalationManager := &EscalationManager{
		config:   &config.EscalationConfig,
		policies: make(map[string]*EscalationPolicy),
		executor: NewEscalationExecutor(config.EscalationConfig),
		tracker:  NewEscalationTracker(config.EscalationConfig),
		metrics:  metrics,
	}
	
	// Initialize fatigue preventor
	fatiguePreventor := &FatiguePreventor{
		config:    &config.FatiguePreventionConfig,
		analyzer:  NewFatigueAnalyzer(config.FatiguePreventionConfig),
		mitigator: NewFatigueMitigator(config.FatiguePreventionConfig),
		metrics:   metrics,
	}
	
	// Initialize metrics collector
	metricsCollector := &MetricsCollector{
		metrics:    metrics,
		collectors: make(map[string]MetricCollector),
	}
	
	// Create service instance
	service := &OnCallRunbooksService{
		incidentManager:   incidentManager,
		runbookManager:    runbookManager,
		pagingManager:     pagingManager,
		alertManager:      alertManager,
		escalationManager: escalationManager,
		fatiguePreventor:  fatiguePreventor,
		metricsCollector:  metricsCollector,
		metrics:           metrics,
		tracer:            tracer,
		config:            config,
	}
	
	// Start HTTP server
	router := mux.NewRouter()
	service.setupRoutes(router)
	
	server := &http.Server{
		Addr:         fmt.Sprintf(":%d", config.Port),
		Handler:      router,
		ReadTimeout:  30 * time.Second,
		WriteTimeout: 30 * time.Second,
		IdleTimeout:  60 * time.Second,
	}
	
	// Start metrics server
	metricsRouter := mux.NewRouter()
	metricsRouter.Handle("/metrics", promhttp.Handler())
	metricsRouter.HandleFunc("/health", service.healthCheck)
	metricsRouter.HandleFunc("/ready", service.readinessCheck)
	
	metricsServer := &http.Server{
		Addr:    fmt.Sprintf(":%d", config.MetricsPort),
		Handler: metricsRouter,
	}
	
	// Start servers
	go func() {
		log.Printf("Starting OnCall Runbooks service on port %d", config.Port)
		if err := server.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("Server failed to start: %v", err)
		}
	}()
	
	go func() {
		log.Printf("Starting metrics server on port %d", config.MetricsPort)
		if err := metricsServer.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("Metrics server failed to start: %v", err)
		}
	}()
	
	// Start background services
	go service.startIncidentProcessor()
	go service.startAlertProcessor()
	go service.startPagingProcessor()
	go service.startEscalationProcessor()
	go service.startFatigueMonitor()
	go service.startMetricsCollection()
	
	// Wait for interrupt signal
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt, syscall.SIGTERM)
	<-c
	
	log.Println("Shutting down OnCall Runbooks service...")
	
	// Graceful shutdown
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()
	
	server.Shutdown(ctx)
	metricsServer.Shutdown(ctx)
	
	log.Println("OnCall Runbooks service stopped")
}

func (s *OnCallRunbooksService) setupRoutes(router *mux.Router) {
	// API routes
	api := router.PathPrefix("/api/v1").Subrouter()
	
	// Incident management endpoints
	api.HandleFunc("/incidents", s.listIncidents).Methods("GET")
	api.HandleFunc("/incidents", s.createIncident).Methods("POST")
	api.HandleFunc("/incidents/{incidentId}", s.getIncident).Methods("GET")
	api.HandleFunc("/incidents/{incidentId}", s.updateIncident).Methods("PUT")
	api.HandleFunc("/incidents/{incidentId}/acknowledge", s.acknowledgeIncident).Methods("POST")
	api.HandleFunc("/incidents/{incidentId}/resolve", s.resolveIncident).Methods("POST")
	api.HandleFunc("/incidents/{incidentId}/escalate", s.escalateIncident).Methods("POST")
	api.HandleFunc("/incidents/{incidentId}/assign", s.assignIncident).Methods("POST")
	
	// Runbook management endpoints
	api.HandleFunc("/runbooks", s.listRunbooks).Methods("GET")
	api.HandleFunc("/runbooks", s.createRunbook).Methods("POST")
	api.HandleFunc("/runbooks/{runbookId}", s.getRunbook).Methods("GET")
	api.HandleFunc("/runbooks/{runbookId}", s.updateRunbook).Methods("PUT")
	api.HandleFunc("/runbooks/{runbookId}/execute", s.executeRunbook).Methods("POST")
	api.HandleFunc("/runbooks/{runbookId}/test", s.testRunbook).Methods("POST")
	api.HandleFunc("/runbooks/suggest", s.suggestRunbooks).Methods("POST")
	
	// Alert management endpoints
	api.HandleFunc("/alerts", s.listAlerts).Methods("GET")
	api.HandleFunc("/alerts", s.receiveAlert).Methods("POST")
	api.HandleFunc("/alerts/{alertId}", s.getAlert).Methods("GET")
	api.HandleFunc("/alerts/{alertId}/silence", s.silenceAlert).Methods("POST")
	api.HandleFunc("/alerts/{alertId}/unsilence", s.unsilenceAlert).Methods("POST")
	api.HandleFunc("/alerts/groups", s.getAlertGroups).Methods("GET")
	
	// Paging and notification endpoints
	api.HandleFunc("/paging/rules", s.listPagingRules).Methods("GET")
	api.HandleFunc("/paging/rules", s.createPagingRule).Methods("POST")
	api.HandleFunc("/paging/rules/{ruleId}", s.updatePagingRule).Methods("PUT")
	api.HandleFunc("/paging/test", s.testPaging).Methods("POST")
	api.HandleFunc("/paging/channels", s.listPagingChannels).Methods("GET")
	
	// On-call schedule endpoints
	api.HandleFunc("/schedules", s.listSchedules).Methods("GET")
	api.HandleFunc("/schedules", s.createSchedule).Methods("POST")
	api.HandleFunc("/schedules/{scheduleId}", s.getSchedule).Methods("GET")
	api.HandleFunc("/schedules/{scheduleId}", s.updateSchedule).Methods("PUT")
	api.HandleFunc("/schedules/{scheduleId}/oncall", s.getCurrentOnCall).Methods("GET")
	api.HandleFunc("/schedules/{scheduleId}/override", s.createOverride).Methods("POST")
	
	// Escalation policy endpoints
	api.HandleFunc("/escalation/policies", s.listEscalationPolicies).Methods("GET")
	api.HandleFunc("/escalation/policies", s.createEscalationPolicy).Methods("POST")
	api.HandleFunc("/escalation/policies/{policyId}", s.getEscalationPolicy).Methods("GET")
	api.HandleFunc("/escalation/policies/{policyId}", s.updateEscalationPolicy).Methods("PUT")
	
	// Fatigue prevention endpoints
	api.HandleFunc("/fatigue/analysis", s.getFatigueAnalysis).Methods("GET")
	api.HandleFunc("/fatigue/metrics", s.getFatigueMetrics).Methods("GET")
	api.HandleFunc("/fatigue/recommendations", s.getFatigueRecommendations).Methods("GET")
	
	// Metrics and reporting endpoints
	api.HandleFunc("/metrics/incidents", s.getIncidentMetrics).Methods("GET")
	api.HandleFunc("/metrics/alerts", s.getAlertMetrics).Methods("GET")
	api.HandleFunc("/metrics/response-times", s.getResponseTimeMetrics).Methods("GET")
	api.HandleFunc("/reports/post-mortems", s.listPostMortems).Methods("GET")
	api.HandleFunc("/reports/post-mortems", s.createPostMortem).Methods("POST")
	
	// Health and status
	api.HandleFunc("/health", s.healthCheck).Methods("GET")
	api.HandleFunc("/ready", s.readinessCheck).Methods("GET")
	api.HandleFunc("/status", s.getStatus).Methods("GET")
}

func (s *OnCallRunbooksService) createIncident(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "create_incident")
	defer span.End()
	
	var request struct {
		Title            string   `json:"title"`
		Description      string   `json:"description"`
		Severity         string   `json:"severity"`
		Type             string   `json:"type"`
		AffectedServices []string `json:"affected_services"`
		Source           string   `json:"source"`
	}
	
	if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
		http.Error(w, "Invalid request", http.StatusBadRequest)
		return
	}
	
	// Create incident
	incident, err := s.incidentManager.CreateIncident(ctx, &request)
	if err != nil {
		s.metrics.IncidentsTotal.WithLabelValues("failed", "creation_error").Inc()
		span.RecordError(err)
		http.Error(w, fmt.Sprintf("Failed to create incident: %v", err), http.StatusInternalServerError)
		return
	}
	
	s.metrics.IncidentsTotal.WithLabelValues("created", request.Severity).Inc()
	
	// Trigger paging if needed
	if err := s.pagingManager.ProcessIncident(ctx, incident); err != nil {
		log.Printf("Failed to process paging for incident %s: %v", incident.ID, err)
	}
	
	// Suggest runbooks
	runbooks := s.runbookManager.SuggestRunbooks(ctx, incident)
	incident.Runbooks = runbooks
	
	span.SetAttributes(
		attribute.String("incident_id", incident.ID),
		attribute.String("severity", request.Severity),
		attribute.String("type", request.Type),
		attribute.Int("affected_services", len(request.AffectedServices)),
	)
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(incident)
}

func (s *OnCallRunbooksService) receiveAlert(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "receive_alert")
	defer span.End()
	
	var alert Alert
	if err := json.NewDecoder(r.Body).Decode(&alert); err != nil {
		http.Error(w, "Invalid alert", http.StatusBadRequest)
		return
	}
	
	start := time.Now()
	
	// Process alert
	processedAlert, err := s.alertManager.ProcessAlert(ctx, &alert)
	if err != nil {
		s.metrics.AlertsTotal.WithLabelValues("failed", "processing_error").Inc()
		span.RecordError(err)
		http.Error(w, fmt.Sprintf("Failed to process alert: %v", err), http.StatusInternalServerError)
		return
	}
	
	processingTime := time.Since(start)
	s.metrics.AlertsTotal.WithLabelValues("processed", processedAlert.Severity).Inc()
	s.metrics.AlertProcessingTime.WithLabelValues(processedAlert.Source).Observe(processingTime.Seconds())
	
	// Check for fatigue prevention
	if s.fatiguePreventor.ShouldSuppressAlert(ctx, processedAlert) {
		s.metrics.AlertsTotal.WithLabelValues("suppressed", "fatigue_prevention").Inc()
		span.SetAttributes(attribute.Bool("suppressed", true))
		
		w.Header().Set("Content-Type", "application/json")
		json.NewEncoder(w).Encode(map[string]interface{}{
			"status":     "suppressed",
			"reason":     "fatigue_prevention",
			"alert_id":   processedAlert.ID,
		})
		return
	}
	
	// Trigger paging if needed
	if err := s.pagingManager.ProcessAlert(ctx, processedAlert); err != nil {
		log.Printf("Failed to process paging for alert %s: %v", processedAlert.ID, err)
	}
	
	span.SetAttributes(
		attribute.String("alert_id", processedAlert.ID),
		attribute.String("severity", processedAlert.Severity),
		attribute.String("source", processedAlert.Source),
		attribute.Float64("processing_time_seconds", processingTime.Seconds()),
	)
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(processedAlert)
}

func (s *OnCallRunbooksService) executeRunbook(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "execute_runbook")
	defer span.End()
	
	vars := mux.Vars(r)
	runbookID := vars["runbookId"]
	
	var request struct {
		IncidentID   string                 `json:"incident_id,omitempty"`
		Parameters   map[string]interface{} `json:"parameters,omitempty"`
		ExecutedBy   string                 `json:"executed_by"`
		DryRun       bool                   `json:"dry_run,omitempty"`
	}
	
	if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
		http.Error(w, "Invalid request", http.StatusBadRequest)
		return
	}
	
	// Execute runbook
	start := time.Now()
	execution, err := s.runbookManager.ExecuteRunbook(ctx, runbookID, &request)
	if err != nil {
		s.metrics.RunbookExecutions.WithLabelValues("failed", "execution_error").Inc()
		span.RecordError(err)
		http.Error(w, fmt.Sprintf("Failed to execute runbook: %v", err), http.StatusInternalServerError)
		return
	}
	
	executionTime := time.Since(start)
	s.metrics.RunbookExecutions.WithLabelValues("success", "completed").Inc()
	
	span.SetAttributes(
		attribute.String("runbook_id", runbookID),
		attribute.String("execution_id", execution.ID),
		attribute.String("executed_by", request.ExecutedBy),
		attribute.Bool("dry_run", request.DryRun),
		attribute.Float64("execution_time_seconds", executionTime.Seconds()),
	)
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(execution)
}

func (s *OnCallRunbooksService) getFatigueAnalysis(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "get_fatigue_analysis")
	defer span.End()
	
	// Get fatigue analysis
	analysis := s.fatiguePreventor.AnalyzeFatigue(ctx)
	
	// Update fatigue score metric
	s.metrics.FatigueScore.WithLabelValues("overall").Set(analysis.OverallScore)
	for team, score := range analysis.TeamScores {
		s.metrics.FatigueScore.WithLabelValues(team).Set(score)
	}
	
	span.SetAttributes(
		attribute.Float64("overall_fatigue_score", analysis.OverallScore),
		attribute.Int("high_fatigue_teams", analysis.HighFatigueTeams),
		attribute.Int("recommendations_count", len(analysis.Recommendations)),
	)
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(analysis)
}

func (s *OnCallRunbooksService) startIncidentProcessor() {
	log.Println("Starting incident processor...")
	
	ticker := time.NewTicker(30 * time.Second)
	defer ticker.Stop()
	
	for {
		select {
		case <-ticker.C:
			s.processIncidentUpdates()
		}
	}
}

func (s *OnCallRunbooksService) startAlertProcessor() {
	log.Println("Starting alert processor...")
	
	ticker := time.NewTicker(10 * time.Second)
	defer ticker.Stop()
	
	for {
		select {
		case <-ticker.C:
			s.processAlertGroups()
		}
	}
}

func (s *OnCallRunbooksService) startPagingProcessor() {
	log.Println("Starting paging processor...")
	
	ticker := time.NewTicker(5 * time.Second)
	defer ticker.Stop()
	
	for {
		select {
		case <-ticker.C:
			s.processPendingPages()
		}
	}
}

func (s *OnCallRunbooksService) startEscalationProcessor() {
	log.Println("Starting escalation processor...")
	
	ticker := time.NewTicker(1 * time.Minute)
	defer ticker.Stop()
	
	for {
		select {
		case <-ticker.C:
			s.processEscalations()
		}
	}
}

func (s *OnCallRunbooksService) startFatigueMonitor() {
	log.Println("Starting fatigue monitor...")
	
	ticker := time.NewTicker(5 * time.Minute)
	defer ticker.Stop()
	
	for {
		select {
		case <-ticker.C:
			s.monitorFatigue()
		}
	}
}

func (s *OnCallRunbooksService) startMetricsCollection() {
	log.Println("Starting metrics collection...")
	
	ticker := time.NewTicker(1 * time.Minute)
	defer ticker.Stop()
	
	for {
		select {
		case <-ticker.C:
			s.collectMetrics()
		}
	}
}

func (s *OnCallRunbooksService) processIncidentUpdates() {
	// Process incident status updates and SLA tracking
	for _, incident := range s.incidentManager.GetActiveIncidents() {
		// Check response time SLA
		if incident.AcknowledgedAt == nil {
			responseTime := time.Since(incident.CreatedAt)
			target := s.getResponseTimeTarget(incident.Severity)
			if responseTime > target {
				s.escalationManager.TriggerEscalation(context.Background(), incident.ID, "response_time_exceeded")
			}
		}
		
		// Check resolution time SLA
		if incident.ResolvedAt == nil && incident.AcknowledgedAt != nil {
			resolutionTime := time.Since(*incident.AcknowledgedAt)
			target := s.getResolutionTimeTarget(incident.Severity)
			if resolutionTime > target {
				s.escalationManager.TriggerEscalation(context.Background(), incident.ID, "resolution_time_exceeded")
			}
		}
	}
}

func (s *OnCallRunbooksService) processAlertGroups() {
	// Process alert grouping and deduplication
	groups := s.alertManager.GetAlertGroups()
	
	for _, group := range groups {
		if s.alertManager.ShouldNotifyGroup(group) {
			if err := s.pagingManager.ProcessAlertGroup(context.Background(), group); err != nil {
				log.Printf("Failed to process alert group %s: %v", group.ID, err)
			}
		}
	}
}

func (s *OnCallRunbooksService) processPendingPages() {
	// Process pending page deliveries and retries
	pendingPages := s.pagingManager.GetPendingPages()
	
	for _, page := range pendingPages {
		start := time.Now()
		if err := s.pagingManager.DeliverPage(context.Background(), page); err != nil {
			log.Printf("Failed to deliver page %s: %v", page.ID, err)
		} else {
			deliveryTime := time.Since(start)
			s.metrics.PagingDeliveryTime.WithLabelValues(page.Channel).Observe(deliveryTime.Seconds())
		}
	}
}

func (s *OnCallRunbooksService) processEscalations() {
	// Process escalation timeouts and triggers
	escalations := s.escalationManager.GetActiveEscalations()
	
	for _, escalation := range escalations {
		if s.escalationManager.ShouldEscalate(escalation) {
			if err := s.escalationManager.ExecuteEscalation(context.Background(), escalation.ID); err != nil {
				log.Printf("Failed to execute escalation %s: %v", escalation.ID, err)
			} else {
				s.metrics.EscalationEvents.WithLabelValues("executed", escalation.Policy).Inc()
			}
		}
	}
}

func (s *OnCallRunbooksService) monitorFatigue() {
	// Monitor alert fatigue and take preventive actions
	analysis := s.fatiguePreventor.AnalyzeFatigue(context.Background())
	
	// Update fatigue metrics
	s.metrics.FatigueScore.WithLabelValues("overall").Set(analysis.OverallScore)
	
	// Take mitigation actions if needed
	if analysis.RequiresMitigation {
		if err := s.fatiguePreventor.ApplyMitigations(context.Background(), analysis); err != nil {
			log.Printf("Failed to apply fatigue mitigations: %v", err)
		}
	}
}

func (s *OnCallRunbooksService) collectMetrics() {
	// Collect and update various metrics
	
	// Response time metrics
	responseTimeStats := s.incidentManager.GetResponseTimeStats()
	for severity, avgTime := range responseTimeStats {
		s.metrics.ResponseTime.WithLabelValues(severity).Observe(avgTime.Seconds())
	}
	
	// Resolution time metrics
	resolutionTimeStats := s.incidentManager.GetResolutionTimeStats()
	for severity, avgTime := range resolutionTimeStats {
		s.metrics.ResolutionTime.WithLabelValues(severity).Observe(avgTime.Seconds())
	}
	
	// On-call coverage metrics
	coverageStats := s.getOnCallCoverageStats()
	for team, coverage := range coverageStats {
		s.metrics.OnCallCoverage.WithLabelValues(team).Set(coverage)
	}
}

// Helper functions and placeholder implementations
func loadConfig() *Config {
	return &Config{
		Port:        8080,
		MetricsPort: 9090,
		IncidentConfig: IncidentConfig{
			AutoClassification: true,
			AutoAssignment:     true,
			StatusUpdateInterval: 5 * time.Minute,
			ResponseTimeTargets: map[string]time.Duration{
				"critical": 5 * time.Minute,
				"high":     15 * time.Minute,
				"medium":   1 * time.Hour,
				"low":      4 * time.Hour,
			},
			ResolutionTimeTargets: map[string]time.Duration{
				"critical": 1 * time.Hour,
				"high":     4 * time.Hour,
				"medium":   24 * time.Hour,
				"low":      72 * time.Hour,
			},
		},
		RunbookConfig: RunbookConfig{
			Repository:          "git",
			AutoSuggestion:      true,
			VersionControl:      true,
			ApprovalRequired:    true,
			TestingRequired:     true,
			UpdateNotifications: true,
			SearchEnabled:       true,
		},
		PagingConfig: PagingConfig{
			PagingEnabled:         true,
			EscalationDelay:       15 * time.Minute,
			MaxRetries:            3,
			RetryInterval:         5 * time.Minute,
			AcknowledgmentTimeout: 30 * time.Minute,
		},
		AlertConfig: AlertConfig{
			DeduplicationEnabled: true,
			DeduplicationWindow:  5 * time.Minute,
		},
		EscalationConfig: EscalationConfig{
			EscalationTimeout:   30 * time.Minute,
			ManagerEscalation:   true,
			ExecutiveEscalation: true,
			ExternalEscalation:  false,
		},
		FatiguePreventionConfig: FatiguePreventionConfig{
			Enabled:                true,
			AlertVelocityThreshold: 50,
			AlertVelocityWindow:    1 * time.Hour,
			SilenceThreshold:       10,
			SilenceDuration:        30 * time.Minute,
			IntelligentGrouping:    true,
			AdaptiveThresholds:     true,
		},
		NotificationConfig: NotificationConfig{
			RateLimiting:     true,
			DeliveryTracking: true,
			FailureRetry:     true,
		},
		SchedulingConfig: SchedulingConfig{
			ScheduleEnabled:     true,
			HandoffReminders:    true,
			CoverageValidation:  true,
			TimeZoneSupport:     true,
		},
	}
}

func initializeMetrics() *OnCallMetrics {
	metrics := &OnCallMetrics{
		IncidentsTotal: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_oncall_incidents_total",
				Help: "Total incidents",
			},
			[]string{"status", "severity"},
		),
		IncidentDuration: prometheus.NewHistogramVec(
			prometheus.HistogramOpts{
				Name: "atlasmesh_oncall_incident_duration_seconds",
				Help: "Incident duration",
			},
			[]string{"severity", "type"},
		),
		AlertsTotal: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_oncall_alerts_total",
				Help: "Total alerts",
			},
			[]string{"status", "severity"},
		),
		AlertProcessingTime: prometheus.NewHistogramVec(
			prometheus.HistogramOpts{
				Name: "atlasmesh_oncall_alert_processing_duration_seconds",
				Help: "Alert processing duration",
			},
			[]string{"source"},
		),
		PagingDeliveryTime: prometheus.NewHistogramVec(
			prometheus.HistogramOpts{
				Name: "atlasmesh_oncall_paging_delivery_duration_seconds",
				Help: "Paging delivery duration",
			},
			[]string{"channel"},
		),
		EscalationEvents: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_oncall_escalation_events_total",
				Help: "Escalation events",
			},
			[]string{"status", "policy"},
		),
		RunbookExecutions: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_oncall_runbook_executions_total",
				Help: "Runbook executions",
			},
			[]string{"status", "result"},
		),
		FatigueScore: prometheus.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "atlasmesh_oncall_fatigue_score",
				Help: "Alert fatigue score",
			},
			[]string{"team"},
		),
		ResponseTime: prometheus.NewHistogramVec(
			prometheus.HistogramOpts{
				Name: "atlasmesh_oncall_response_time_seconds",
				Help: "Incident response time",
			},
			[]string{"severity"},
		),
		ResolutionTime: prometheus.NewHistogramVec(
			prometheus.HistogramOpts{
				Name: "atlasmesh_oncall_resolution_time_seconds",
				Help: "Incident resolution time",
			},
			[]string{"severity"},
		),
		OnCallCoverage: prometheus.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "atlasmesh_oncall_coverage_percentage",
				Help: "On-call coverage percentage",
			},
			[]string{"team"},
		),
	}
	
	prometheus.MustRegister(
		metrics.IncidentsTotal,
		metrics.IncidentDuration,
		metrics.AlertsTotal,
		metrics.AlertProcessingTime,
		metrics.PagingDeliveryTime,
		metrics.EscalationEvents,
		metrics.RunbookExecutions,
		metrics.FatigueScore,
		metrics.ResponseTime,
		metrics.ResolutionTime,
		metrics.OnCallCoverage,
	)
	
	return metrics
}

func (s *OnCallRunbooksService) healthCheck(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "healthy"})
}

func (s *OnCallRunbooksService) readinessCheck(w http.ResponseWriter, r *http.Request) {
	// Check if all components are ready
	ready := s.incidentManager.IsReady() &&
		s.runbookManager.IsReady() &&
		s.pagingManager.IsReady() &&
		s.alertManager.IsReady() &&
		s.escalationManager.IsReady()
	
	if !ready {
		http.Error(w, "Service not ready", http.StatusServiceUnavailable)
		return
	}
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "ready"})
}

func (s *OnCallRunbooksService) getStatus(w http.ResponseWriter, r *http.Request) {
	status := map[string]interface{}{
		"service":            "oncall-runbooks",
		"version":            "1.0.0",
		"incident_manager":   s.incidentManager.GetStatus(),
		"runbook_manager":    s.runbookManager.GetStatus(),
		"paging_manager":     s.pagingManager.GetStatus(),
		"alert_manager":      s.alertManager.GetStatus(),
		"escalation_manager": s.escalationManager.GetStatus(),
		"fatigue_preventor":  s.fatiguePreventor.GetStatus(),
	}
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(status)
}

// Placeholder implementations for interfaces and methods
type IncidentClassifier interface{ Classify(incident *Incident) error }
type IncidentAssigner interface{ Assign(incident *Incident) error }
type RunbookRepository interface{ Store(runbook *Runbook) error }
type RunbookSuggester interface{ Suggest(incident *Incident) []string }
type RunbookExecutor interface{ Execute(runbookID string, params map[string]interface{}) (*RunbookExecution, error) }
type PagingScheduler interface{ Schedule(page *Page) error }
type NotificationDeliverer interface{ Deliver(notification *Notification) error }
type AlertProcessor interface{ Process(alert *Alert) (*Alert, error) }
type AlertGrouper interface{ Group(alerts []*Alert) []*AlertGroup }
type AlertSuppressor interface{ Suppress(alert *Alert) bool }
type EscalationExecutor interface{ Execute(escalationID string) error }
type EscalationTracker interface{ Track(escalation *Escalation) error }
type FatigueAnalyzer interface{ Analyze() *FatigueAnalysis }
type FatigueMitigator interface{ Mitigate(analysis *FatigueAnalysis) error }
type MetricCollector interface{ Collect() map[string]float64 }

type RunbookExecution struct {
	ID       string    `json:"id"`
	Status   string    `json:"status"`
	StartTime time.Time `json:"start_time"`
	EndTime  *time.Time `json:"end_time,omitempty"`
}

type Page struct {
	ID      string `json:"id"`
	Channel string `json:"channel"`
	Message string `json:"message"`
}

type Notification struct {
	ID      string `json:"id"`
	Channel string `json:"channel"`
	Content string `json:"content"`
}

type AlertGroup struct {
	ID     string   `json:"id"`
	Alerts []*Alert `json:"alerts"`
}

type Escalation struct {
	ID     string `json:"id"`
	Policy string `json:"policy"`
}

type FatigueAnalysis struct {
	OverallScore       float64            `json:"overall_score"`
	TeamScores         map[string]float64 `json:"team_scores"`
	HighFatigueTeams   int                `json:"high_fatigue_teams"`
	RequiresMitigation bool               `json:"requires_mitigation"`
	Recommendations    []string           `json:"recommendations"`
}

func NewIncidentClassifier(config IncidentConfig) IncidentClassifier { return nil }
func NewIncidentAssigner(config IncidentConfig) IncidentAssigner { return nil }
func NewRunbookRepository(config RunbookConfig) RunbookRepository { return nil }
func NewRunbookSuggester(config RunbookConfig) RunbookSuggester { return nil }
func NewRunbookExecutor(config RunbookConfig) RunbookExecutor { return nil }
func NewPagingScheduler(config PagingConfig) PagingScheduler { return nil }
func NewNotificationDeliverer(config NotificationConfig) NotificationDeliverer { return nil }
func NewAlertProcessor(config AlertConfig) AlertProcessor { return nil }
func NewAlertGrouper(config AlertConfig) AlertGrouper { return nil }
func NewAlertSuppressor(config AlertConfig) AlertSuppressor { return nil }
func NewEscalationExecutor(config EscalationConfig) EscalationExecutor { return nil }
func NewEscalationTracker(config EscalationConfig) EscalationTracker { return nil }
func NewFatigueAnalyzer(config FatiguePreventionConfig) FatigueAnalyzer { return nil }
func NewFatigueMitigator(config FatiguePreventionConfig) FatigueMitigator { return nil }

// Placeholder method implementations
func (im *IncidentManager) CreateIncident(ctx context.Context, req interface{}) (*Incident, error) {
	incident := &Incident{
		ID:        fmt.Sprintf("inc_%d", time.Now().Unix()),
		Status:    "open",
		CreatedAt: time.Now(),
	}
	im.incidents[incident.ID] = incident
	return incident, nil
}
func (im *IncidentManager) GetActiveIncidents() []*Incident { return []*Incident{} }
func (im *IncidentManager) GetResponseTimeStats() map[string]time.Duration { return map[string]time.Duration{} }
func (im *IncidentManager) GetResolutionTimeStats() map[string]time.Duration { return map[string]time.Duration{} }
func (im *IncidentManager) IsReady() bool { return true }
func (im *IncidentManager) GetStatus() map[string]interface{} { return map[string]interface{}{} }

func (rm *RunbookManager) SuggestRunbooks(ctx context.Context, incident *Incident) []string { return []string{} }
func (rm *RunbookManager) ExecuteRunbook(ctx context.Context, runbookID string, req interface{}) (*RunbookExecution, error) {
	execution := &RunbookExecution{
		ID:        fmt.Sprintf("exec_%d", time.Now().Unix()),
		Status:    "running",
		StartTime: time.Now(),
	}
	return execution, nil
}
func (rm *RunbookManager) IsReady() bool { return true }
func (rm *RunbookManager) GetStatus() map[string]interface{} { return map[string]interface{}{} }

func (pm *PagingManager) ProcessIncident(ctx context.Context, incident *Incident) error { return nil }
func (pm *PagingManager) ProcessAlert(ctx context.Context, alert *Alert) error { return nil }
func (pm *PagingManager) ProcessAlertGroup(ctx context.Context, group *AlertGroup) error { return nil }
func (pm *PagingManager) GetPendingPages() []*Page { return []*Page{} }
func (pm *PagingManager) DeliverPage(ctx context.Context, page *Page) error { return nil }
func (pm *PagingManager) IsReady() bool { return true }
func (pm *PagingManager) GetStatus() map[string]interface{} { return map[string]interface{}{} }

func (am *AlertManager) ProcessAlert(ctx context.Context, alert *Alert) (*Alert, error) {
	alert.ID = fmt.Sprintf("alert_%d", time.Now().Unix())
	am.alerts[alert.ID] = alert
	return alert, nil
}
func (am *AlertManager) GetAlertGroups() []*AlertGroup { return []*AlertGroup{} }
func (am *AlertManager) ShouldNotifyGroup(group *AlertGroup) bool { return true }
func (am *AlertManager) IsReady() bool { return true }
func (am *AlertManager) GetStatus() map[string]interface{} { return map[string]interface{}{} }

func (em *EscalationManager) TriggerEscalation(ctx context.Context, incidentID, reason string) error { return nil }
func (em *EscalationManager) GetActiveEscalations() []*Escalation { return []*Escalation{} }
func (em *EscalationManager) ShouldEscalate(escalation *Escalation) bool { return false }
func (em *EscalationManager) ExecuteEscalation(ctx context.Context, escalationID string) error { return nil }
func (em *EscalationManager) IsReady() bool { return true }
func (em *EscalationManager) GetStatus() map[string]interface{} { return map[string]interface{}{} }

func (fp *FatiguePreventor) ShouldSuppressAlert(ctx context.Context, alert *Alert) bool { return false }
func (fp *FatiguePreventor) AnalyzeFatigue(ctx context.Context) *FatigueAnalysis {
	return &FatigueAnalysis{
		OverallScore:       0.75,
		TeamScores:         map[string]float64{"team1": 0.8, "team2": 0.7},
		HighFatigueTeams:   1,
		RequiresMitigation: false,
		Recommendations:    []string{},
	}
}
func (fp *FatiguePreventor) ApplyMitigations(ctx context.Context, analysis *FatigueAnalysis) error { return nil }
func (fp *FatiguePreventor) GetStatus() map[string]interface{} { return map[string]interface{}{} }

func (s *OnCallRunbooksService) getResponseTimeTarget(severity string) time.Duration {
	if target, exists := s.config.IncidentConfig.ResponseTimeTargets[severity]; exists {
		return target
	}
	return 1 * time.Hour // Default
}

func (s *OnCallRunbooksService) getResolutionTimeTarget(severity string) time.Duration {
	if target, exists := s.config.IncidentConfig.ResolutionTimeTargets[severity]; exists {
		return target
	}
	return 24 * time.Hour // Default
}

func (s *OnCallRunbooksService) getOnCallCoverageStats() map[string]float64 {
	return map[string]float64{"team1": 0.95, "team2": 0.88, "team3": 0.92}
}

// Placeholder implementations for remaining handlers
func (s *OnCallRunbooksService) listIncidents(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) getIncident(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) updateIncident(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) acknowledgeIncident(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) resolveIncident(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) escalateIncident(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) assignIncident(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) listRunbooks(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) createRunbook(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) getRunbook(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) updateRunbook(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) testRunbook(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) suggestRunbooks(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) listAlerts(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) getAlert(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) silenceAlert(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) unsilenceAlert(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) getAlertGroups(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) listPagingRules(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) createPagingRule(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) updatePagingRule(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) testPaging(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) listPagingChannels(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) listSchedules(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) createSchedule(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) getSchedule(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) updateSchedule(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) getCurrentOnCall(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) createOverride(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) listEscalationPolicies(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) createEscalationPolicy(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) getEscalationPolicy(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) updateEscalationPolicy(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) getFatigueMetrics(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) getFatigueRecommendations(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) getIncidentMetrics(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) getAlertMetrics(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) getResponseTimeMetrics(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) listPostMortems(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *OnCallRunbooksService) createPostMortem(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}
