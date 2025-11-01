package budget

import (
	"sync"
	"time"
)

// Tracker manages variant budget state and history.
type Tracker struct {
	config          Config
	currentBudgets  map[string]*BudgetStatus
	history         []HistoricalEntry
	mu              sync.RWMutex
}

// Config holds budget configuration.
type Config struct {
	VehicleBudgetLimit  float64
	SectorBudgetLimit   float64
	PlatformBudgetLimit float64
	TestBudgetLimit     float64
}

// BudgetStatus represents current budget consumption for a dimension.
type BudgetStatus struct {
	Dimension       string    `json:"dimension"`
	CurrentDelta    float64   `json:"current_delta_pct"`
	Limit           float64   `json:"limit_pct"`
	SoftLimit       float64   `json:"soft_limit_pct"`
	TrendGrowth     float64   `json:"trend_growth_pct"`
	Status          string    `json:"status"` // "healthy", "warning", "violation"
	LastUpdated     time.Time `json:"last_updated"`
	ViolationReason string    `json:"violation_reason,omitempty"`
}

// HistoricalEntry records budget status at a point in time.
type HistoricalEntry struct {
	Timestamp   time.Time               `json:"timestamp"`
	CommitSHA   string                  `json:"commit_sha"`
	Budgets     map[string]*BudgetStatus `json:"budgets"`
	EventType   string                  `json:"event_type"` // "analysis", "violation", "exception"
}

// OverallStatus represents aggregate budget status.
type OverallStatus struct {
	AllDimensions   map[string]*BudgetStatus `json:"all_dimensions"`
	HealthyCount    int                      `json:"healthy_count"`
	WarningCount    int                      `json:"warning_count"`
	ViolationCount  int                      `json:"violation_count"`
	OverallStatus   string                   `json:"overall_status"`
	LastAnalysis    time.Time                `json:"last_analysis"`
}

// NewTracker creates a new budget tracker.
func NewTracker(cfg Config) *Tracker {
	return &Tracker{
		config: cfg,
		currentBudgets: map[string]*BudgetStatus{
			"vehicle":  {Dimension: "vehicle", Limit: cfg.VehicleBudgetLimit, SoftLimit: cfg.VehicleBudgetLimit * 0.6, Status: "healthy"},
			"sector":   {Dimension: "sector", Limit: cfg.SectorBudgetLimit, SoftLimit: cfg.SectorBudgetLimit * 0.6, Status: "healthy"},
			"platform": {Dimension: "platform", Limit: cfg.PlatformBudgetLimit, SoftLimit: cfg.PlatformBudgetLimit * 0.6, Status: "healthy"},
			"test":     {Dimension: "test", Limit: cfg.TestBudgetLimit, SoftLimit: cfg.TestBudgetLimit * 0.8, Status: "healthy"},
		},
		history: []HistoricalEntry{},
	}
}

// UpdateBudget updates the budget status for a dimension.
func (t *Tracker) UpdateBudget(dimension string, deltaPct float64, commitSHA string) {
	t.mu.Lock()
	defer t.mu.Unlock()

	budget, exists := t.currentBudgets[dimension]
	if !exists {
		return
	}

	budget.CurrentDelta = deltaPct
	budget.LastUpdated = time.Now()

	// Determine status
	if deltaPct >= budget.Limit {
		budget.Status = "violation"
		budget.ViolationReason = "exceeded hard limit"
	} else if deltaPct >= budget.SoftLimit {
		budget.Status = "warning"
		budget.ViolationReason = "approaching soft limit"
	} else {
		budget.Status = "healthy"
		budget.ViolationReason = ""
	}

	// Record historical entry
	t.recordHistory(commitSHA, "analysis")
}

// GetVehicleBudgetStatus returns current vehicle budget status.
func (t *Tracker) GetVehicleBudgetStatus() *BudgetStatus {
	t.mu.RLock()
	defer t.mu.RUnlock()
	return t.currentBudgets["vehicle"]
}

// GetSectorBudgetStatus returns current sector budget status.
func (t *Tracker) GetSectorBudgetStatus() *BudgetStatus {
	t.mu.RLock()
	defer t.mu.RUnlock()
	return t.currentBudgets["sector"]
}

// GetPlatformBudgetStatus returns current platform budget status.
func (t *Tracker) GetPlatformBudgetStatus() *BudgetStatus {
	t.mu.RLock()
	defer t.mu.RUnlock()
	return t.currentBudgets["platform"]
}

// GetOverallBudgetStatus returns aggregate budget status across all dimensions.
func (t *Tracker) GetOverallBudgetStatus() *OverallStatus {
	t.mu.RLock()
	defer t.mu.RUnlock()

	var healthy, warning, violation int
	for _, budget := range t.currentBudgets {
		switch budget.Status {
		case "healthy":
			healthy++
		case "warning":
			warning++
		case "violation":
			violation++
		}
	}

	overallStatus := "healthy"
	if violation > 0 {
		overallStatus = "violation"
	} else if warning > 0 {
		overallStatus = "warning"
	}

	lastAnalysis := time.Time{}
	for _, budget := range t.currentBudgets {
		if budget.LastUpdated.After(lastAnalysis) {
			lastAnalysis = budget.LastUpdated
		}
	}

	// Create a copy of current budgets to avoid race conditions
	allDimensions := make(map[string]*BudgetStatus)
	for k, v := range t.currentBudgets {
		budgetCopy := *v
		allDimensions[k] = &budgetCopy
	}

	return &OverallStatus{
		AllDimensions:  allDimensions,
		HealthyCount:   healthy,
		WarningCount:   warning,
		ViolationCount: violation,
		OverallStatus:  overallStatus,
		LastAnalysis:   lastAnalysis,
	}
}

// GetHistory returns budget history.
func (t *Tracker) GetHistory(limit int) []HistoricalEntry {
	t.mu.RLock()
	defer t.mu.RUnlock()

	if limit <= 0 || limit > len(t.history) {
		limit = len(t.history)
	}

	// Return most recent entries
	startIdx := len(t.history) - limit
	if startIdx < 0 {
		startIdx = 0
	}

	history := make([]HistoricalEntry, limit)
	copy(history, t.history[startIdx:])
	return history
}

// recordHistory records current budget state in history.
func (t *Tracker) recordHistory(commitSHA, eventType string) {
	// Create snapshot of current budgets
	budgets := make(map[string]*BudgetStatus)
	for k, v := range t.currentBudgets {
		budgetCopy := *v
		budgets[k] = &budgetCopy
	}

	entry := HistoricalEntry{
		Timestamp: time.Now(),
		CommitSHA: commitSHA,
		Budgets:   budgets,
		EventType: eventType,
	}

	t.history = append(t.history, entry)

	// Keep only last 1000 entries
	if len(t.history) > 1000 {
		t.history = t.history[len(t.history)-1000:]
	}
}

