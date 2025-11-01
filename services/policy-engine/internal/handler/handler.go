package handler

import (
	"context"

	"github.com/atlasmesh/fleet-os/services/policy-engine/internal/service"
	pb "github.com/atlasmesh/fleet-os/services/policy-engine/proto"
)

// PolicyEngineHandler implements the gRPC PolicyEngine service.
type PolicyEngineHandler struct {
	pb.UnimplementedPolicyEngineServer
	service *service.PolicyService
}

// NewPolicyEngineHandler creates a new gRPC handler.
func NewPolicyEngineHandler(svc *service.PolicyService) *PolicyEngineHandler {
	return &PolicyEngineHandler{
		service: svc,
	}
}

// EvaluatePolicy evaluates a policy against input data.
func (h *PolicyEngineHandler) EvaluatePolicy(ctx context.Context, req *pb.EvaluatePolicyRequest) (*pb.EvaluatePolicyResponse, error) {
	return h.service.EvaluatePolicy(ctx, req)
}

// BatchEvaluatePolicy evaluates multiple policies in a single request.
func (h *PolicyEngineHandler) BatchEvaluatePolicy(ctx context.Context, req *pb.BatchEvaluatePolicyRequest) (*pb.BatchEvaluatePolicyResponse, error) {
	return h.service.BatchEvaluatePolicy(ctx, req)
}

// CreatePolicy creates a new policy.
func (h *PolicyEngineHandler) CreatePolicy(ctx context.Context, req *pb.CreatePolicyRequest) (*pb.CreatePolicyResponse, error) {
	return h.service.CreatePolicy(ctx, req)
}

// UpdatePolicy updates an existing policy.
func (h *PolicyEngineHandler) UpdatePolicy(ctx context.Context, req *pb.UpdatePolicyRequest) (*pb.UpdatePolicyResponse, error) {
	// TODO: Implement UpdatePolicy in service layer
	return &pb.UpdatePolicyResponse{}, nil
}

// GetPolicy retrieves a policy by ID.
func (h *PolicyEngineHandler) GetPolicy(ctx context.Context, req *pb.GetPolicyRequest) (*pb.GetPolicyResponse, error) {
	// TODO: Implement GetPolicy in service layer
	return &pb.GetPolicyResponse{}, nil
}

// ListPolicies lists policies with optional filtering.
func (h *PolicyEngineHandler) ListPolicies(ctx context.Context, req *pb.ListPoliciesRequest) (*pb.ListPoliciesResponse, error) {
	// TODO: Implement ListPolicies in service layer
	return &pb.ListPoliciesResponse{}, nil
}

// DeletePolicy deletes a policy.
func (h *PolicyEngineHandler) DeletePolicy(ctx context.Context, req *pb.DeletePolicyRequest) (*pb.DeletePolicyResponse, error) {
	// TODO: Implement DeletePolicy in service layer
	return &pb.DeletePolicyResponse{}, nil
}

// ValidatePolicy validates policy syntax without saving.
func (h *PolicyEngineHandler) ValidatePolicy(ctx context.Context, req *pb.ValidatePolicyRequest) (*pb.ValidatePolicyResponse, error) {
	// TODO: Implement ValidatePolicy in service layer
	return &pb.ValidatePolicyResponse{}, nil
}

// GetPolicyVersion retrieves a specific version of a policy.
func (h *PolicyEngineHandler) GetPolicyVersion(ctx context.Context, req *pb.GetPolicyVersionRequest) (*pb.GetPolicyVersionResponse, error) {
	// TODO: Implement GetPolicyVersion in service layer
	return &pb.GetPolicyVersionResponse{}, nil
}

// ListPolicyVersions lists all versions of a policy.
func (h *PolicyEngineHandler) ListPolicyVersions(ctx context.Context, req *pb.ListPolicyVersionsRequest) (*pb.ListPolicyVersionsResponse, error) {
	// TODO: Implement ListPolicyVersions in service layer
	return &pb.ListPolicyVersionsResponse{}, nil
}

// RollbackPolicy rolls back to a previous policy version.
func (h *PolicyEngineHandler) RollbackPolicy(ctx context.Context, req *pb.RollbackPolicyRequest) (*pb.RollbackPolicyResponse, error) {
	// TODO: Implement RollbackPolicy in service layer
	return &pb.RollbackPolicyResponse{}, nil
}

