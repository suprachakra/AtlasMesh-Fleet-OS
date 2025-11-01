package server

import (
	"fmt"
	"net/http"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promauto"

	"github.com/atlasmesh/fleet-os/services/variant-budget/internal/analyzer"
	"github.com/atlasmesh/fleet-os/services/variant-budget/internal/budget"
	"github.com/atlasmesh/fleet-os/services/variant-budget/internal/enforcer"
)

// Server handles HTTP endpoints for variant budget service.
type Server struct {
	analyzer *analyzer.DeltaAnalyzer
	tracker  *budget.Tracker
	enforcer *enforcer.Enforcer
	metrics  *Metrics
}

// Metrics holds Prometheus metrics for the variant budget service.
type Metrics struct {
	AnalysisDuration   prometheus.Histogram
	BudgetViolations   *prometheus.CounterVec
	BudgetWarnings     *prometheus.CounterVec
	CCBRequests        prometheus.Counter
	CurrentBudgetDelta *prometheus.GaugeVec
}

// New creates a new server instance.
func New(a *analyzer.DeltaAnalyzer, t *budget.Tracker, e *enforcer.Enforcer) *Server {
	metrics := &Metrics{
		AnalysisDuration: promauto.NewHistogram(prometheus.HistogramOpts{
			Namespace: "atlasmesh",
			Subsystem: "variant_budget",
			Name:      "analysis_duration_seconds",
			Help:      "Time taken to analyze code delta",
			Buckets:   []float64{1, 5, 10, 30, 60, 120},
		}),
		BudgetViolations: promauto.NewCounterVec(prometheus.CounterOpts{
			Namespace: "atlasmesh",
			Subsystem: "variant_budget",
			Name:      "violations_total",
			Help:      "Total number of budget violations",
		}, []string{"dimension"}),
		BudgetWarnings: promauto.NewCounterVec(prometheus.CounterOpts{
			Namespace: "atlasmesh",
			Subsystem: "variant_budget",
			Name:      "warnings_total",
			Help:      "Total number of budget warnings",
		}, []string{"dimension"}),
		CCBRequests: promauto.NewCounter(prometheus.CounterOpts{
			Namespace: "atlasmesh",
			Subsystem: "variant_budget",
			Name:      "ccb_requests_total",
			Help:      "Total number of CCB review requests",
		}),
		CurrentBudgetDelta: promauto.NewGaugeVec(prometheus.GaugeOpts{
			Namespace: "atlasmesh",
			Subsystem: "variant_budget",
			Name:      "current_delta_pct",
			Help:      "Current budget delta percentage by dimension",
		}, []string{"dimension"}),
	}

	return &Server{
		analyzer: a,
		tracker:  t,
		enforcer: e,
		metrics:  metrics,
	}
}

// SetupRoutes configures HTTP routes.
func (s *Server) SetupRoutes(router *gin.Engine) {
	api := router.Group("/api/v1/budget")

	api.POST("/analyze", s.handleAnalyze)
	api.GET("/status", s.handleStatus)
	api.GET("/history", s.handleHistory)
	api.GET("/violations", s.handleViolations)
	api.POST("/ccb/request", s.handleCCBRequest)
	api.POST("/ccb/approve", s.handleCCBApprove)
	api.POST("/ccb/reject", s.handleCCBReject)
}

// handleAnalyze performs budget analysis for a commit.
func (s *Server) handleAnalyze(c *gin.Context) {
	var req struct {
		CommitSHA string `json:"commit_sha"`
		Baseline  string `json:"baseline"`
	}

	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": "invalid request", "details": err.Error()})
		return
	}

	start := time.Now()
	
	// Perform delta analysis
	result, err := s.analyzer.Analyze(c.Request.Context(), req.CommitSHA)
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "analysis failed", "details": err.Error()})
		return
	}

	duration := time.Since(start)
	s.metrics.AnalysisDuration.Observe(duration.Seconds())

	// Update budget tracker
	s.tracker.UpdateBudget("vehicle", result.VehicleDelta.CodeDeltaPct, req.CommitSHA)
	s.tracker.UpdateBudget("sector", result.SectorDelta.CodeDeltaPct, req.CommitSHA)
	s.tracker.UpdateBudget("platform", result.PlatformDelta.CodeDeltaPct, req.CommitSHA)
	s.tracker.UpdateBudget("test", result.TestDelta.CodeDeltaPct, req.CommitSHA)

	// Update metrics
	s.metrics.CurrentBudgetDelta.WithLabelValues("vehicle").Set(result.VehicleDelta.CodeDeltaPct)
	s.metrics.CurrentBudgetDelta.WithLabelValues("sector").Set(result.SectorDelta.CodeDeltaPct)
	s.metrics.CurrentBudgetDelta.WithLabelValues("platform").Set(result.PlatformDelta.CodeDeltaPct)
	s.metrics.CurrentBudgetDelta.WithLabelValues("test").Set(result.TestDelta.CodeDeltaPct)

	// Enforce budget policies
	enforcement, err := s.enforcer.Enforce(c.Request.Context(), req.CommitSHA)
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "enforcement failed", "details": err.Error()})
		return
	}

	// Record violations and warnings
	for _, violation := range enforcement.Violations {
		s.metrics.BudgetViolations.WithLabelValues(violation.Dimension).Inc()
	}
	for _, warning := range enforcement.Warnings {
		s.metrics.BudgetWarnings.WithLabelValues(warning.Dimension).Inc()
	}
	if enforcement.CCBTriggered {
		s.metrics.CCBRequests.Inc()
	}

	c.JSON(http.StatusOK, gin.H{
		"analysis":    result,
		"enforcement": enforcement,
		"duration_ms": duration.Milliseconds(),
	})
}

// handleStatus returns current budget status.
func (s *Server) handleStatus(c *gin.Context) {
	status := s.tracker.GetOverallBudgetStatus()
	c.JSON(http.StatusOK, status)
}

// handleHistory returns budget history.
func (s *Server) handleHistory(c *gin.Context) {
	limit := 100
	if limitParam := c.Query("limit"); limitParam != "" {
		fmt.Sscanf(limitParam, "%d", &limit)
	}

	history := s.tracker.GetHistory(limit)
	c.JSON(http.StatusOK, gin.H{
		"history": history,
		"count":   len(history),
	})
}

// handleViolations returns current violations.
func (s *Server) handleViolations(c *gin.Context) {
	status := s.tracker.GetOverallBudgetStatus()
	
	violations := []gin.H{}
	for dimension, budgetStatus := range status.AllDimensions {
		if budgetStatus.Status == "violation" {
			violations = append(violations, gin.H{
				"dimension":        dimension,
				"current_delta":    budgetStatus.CurrentDelta,
				"limit":            budgetStatus.Limit,
				"violation_reason": budgetStatus.ViolationReason,
				"last_updated":     budgetStatus.LastUpdated,
			})
		}
	}

	c.JSON(http.StatusOK, gin.H{
		"violations": violations,
		"count":      len(violations),
	})
}

// handleCCBRequest submits a CCB exception request.
func (s *Server) handleCCBRequest(c *gin.Context) {
	var req struct {
		CommitSHA       string `json:"commit_sha" binding:"required"`
		Requester       string `json:"requester" binding:"required"`
		Justification   string `json:"justification" binding:"required"`
		ImpactAnalysis  string `json:"impact_analysis"`
		RemediationPlan string `json:"remediation_plan"`
	}

	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": "invalid request", "details": err.Error()})
		return
	}

	// Get current violations
	status := s.tracker.GetOverallBudgetStatus()
	violations := []BudgetViolation{}
	for dimension, budgetStatus := range status.AllDimensions {
		if budgetStatus.Status == "violation" {
			violations = append(violations, BudgetViolation{
				Dimension:      dimension,
				CurrentDelta:   budgetStatus.CurrentDelta,
				Limit:          budgetStatus.Limit,
				Excess:         budgetStatus.CurrentDelta - budgetStatus.Limit,
				RequiresAction: "CCB approval",
			})
		}
	}

	ccbRequest := CCBRequest{
		RequestID:       fmt.Sprintf("ccb-%s-%d", req.CommitSHA[:8], time.Now().Unix()),
		CommitSHA:       req.CommitSHA,
		Violations:      violations,
		Requester:       req.Requester,
		Justification:   req.Justification,
		ImpactAnalysis:  req.ImpactAnalysis,
		RemediationPlan: req.RemediationPlan,
		CreatedAt:       time.Now(),
	}

	// Send to CCB webhook
	if err := s.sendCCBWebhook(c.Request.Context(), ccbRequest); err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "failed to notify CCB", "details": err.Error()})
		return
	}

	c.JSON(http.StatusAccepted, gin.H{
		"request_id": ccbRequest.RequestID,
		"status":     "pending_ccb_review",
		"violations": violations,
	})
}

// handleCCBApprove processes CCB approval.
func (s *Server) handleCCBApprove(c *gin.Context) {
	var req struct {
		RequestID     string `json:"request_id" binding:"required"`
		CommitSHA     string `json:"commit_sha" binding:"required"`
		Approver      string `json:"approver" binding:"required"`
		Justification string `json:"justification" binding:"required"`
	}

	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": "invalid request", "details": err.Error()})
		return
	}

	if err := s.enforcer.ApproveException(c.Request.Context(), req.CommitSHA, req.Approver, req.Justification); err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "approval failed", "details": err.Error()})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"request_id": req.RequestID,
		"status":     "approved",
		"approver":   req.Approver,
		"approved_at": time.Now(),
	})
}

// handleCCBReject processes CCB rejection.
func (s *Server) handleCCBReject(c *gin.Context) {
	var req struct {
		RequestID string `json:"request_id" binding:"required"`
		CommitSHA string `json:"commit_sha" binding:"required"`
		Rejector  string `json:"rejector" binding:"required"`
		Reason    string `json:"reason" binding:"required"`
	}

	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": "invalid request", "details": err.Error()})
		return
	}

	if err := s.enforcer.RejectException(c.Request.Context(), req.CommitSHA, req.Rejector, req.Reason); err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "rejection failed", "details": err.Error()})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"request_id": req.RequestID,
		"status":     "rejected",
		"rejector":   req.Rejector,
		"rejected_at": time.Now(),
	})
}

// sendCCBWebhook sends CCB request to configured webhook.
func (s *Server) sendCCBWebhook(ctx context.Context, request enforcer.CCBRequest) error {
	if s.enforcer == nil {
		return nil
	}
	return nil // Webhook handling done in enforcer.triggerCCB
}

// PerformPeriodicAnalysis runs periodic budget analysis.
func (s *Server) PerformPeriodicAnalysis() error {
	// Analyze current state
	result, err := s.analyzer.Analyze(context.Background(), "")
	if err != nil {
		return fmt.Errorf("periodic analysis failed: %w", err)
	}

	// Update tracker
	s.tracker.UpdateBudget("vehicle", result.VehicleDelta.CodeDeltaPct, result.CommitSHA)
	s.tracker.UpdateBudget("sector", result.SectorDelta.CodeDeltaPct, result.CommitSHA)
	s.tracker.UpdateBudget("platform", result.PlatformDelta.CodeDeltaPct, result.CommitSHA)
	s.tracker.UpdateBudget("test", result.TestDelta.CodeDeltaPct, result.CommitSHA)

	// Update metrics
	s.metrics.CurrentBudgetDelta.WithLabelValues("vehicle").Set(result.VehicleDelta.CodeDeltaPct)
	s.metrics.CurrentBudgetDelta.WithLabelValues("sector").Set(result.SectorDelta.CodeDeltaPct)
	s.metrics.CurrentBudgetDelta.WithLabelValues("platform").Set(result.PlatformDelta.CodeDeltaPct)
	s.metrics.CurrentBudgetDelta.WithLabelValues("test").Set(result.TestDelta.CodeDeltaPct)

	return nil
}

// GetPrometheusMetrics returns Prometheus metrics in text format.
func (s *Server) GetPrometheusMetrics() string {
	// TODO: Implement proper Prometheus metrics export
	return "# Variant Budget Metrics\n"
}

