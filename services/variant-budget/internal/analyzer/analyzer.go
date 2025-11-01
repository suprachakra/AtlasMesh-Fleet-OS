package analyzer

import (
	"context"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strings"

	"github.com/go-git/go-git/v5"
	"github.com/go-git/go-git/v5/plumbing"
)

// DeltaAnalyzer performs code delta analysis for variant budget tracking.
type DeltaAnalyzer struct {
	repoPath       string
	baselineBranch string
	exclusions     []string
}

// Config holds configuration for the delta analyzer.
type Config struct {
	RepoPath       string
	BaselineBranch string
	Exclusions     []string
}

// AnalysisResult contains the results of a delta analysis.
type AnalysisResult struct {
	CommitSHA         string                 `json:"commit_sha"`
	BaselineCommit    string                 `json:"baseline_commit"`
	VehicleDelta      DimensionDelta         `json:"vehicle_delta"`
	SectorDelta       DimensionDelta         `json:"sector_delta"`
	PlatformDelta     DimensionDelta         `json:"platform_delta"`
	TestDelta         DimensionDelta         `json:"test_delta"`
	TotalFilesChanged int                    `json:"total_files_changed"`
	Metadata          map[string]interface{} `json:"metadata"`
}

// DimensionDelta represents code delta for a specific agnostic dimension.
type DimensionDelta struct {
	LinesAdded      int     `json:"lines_added"`
	LinesDeleted    int     `json:"lines_deleted"`
	LinesModified   int     `json:"lines_modified"`
	FilesChanged    int     `json:"files_changed"`
	CodeDeltaPct    float64 `json:"code_delta_pct"`
	ComplexityDelta int     `json:"complexity_delta"`
}

// New creates a new DeltaAnalyzer instance.
func New(cfg Config) (*DeltaAnalyzer, error) {
	if cfg.RepoPath == "" {
		cwd, err := os.Getwd()
		if err != nil {
			return nil, fmt.Errorf("failed to get working directory: %w", err)
		}
		cfg.RepoPath = cwd
	}

	if cfg.BaselineBranch == "" {
		cfg.BaselineBranch = "main"
	}

	if cfg.Exclusions == nil {
		cfg.Exclusions = []string{
			"configs/",
			"docs/",
			"*.md",
			"*.yaml",
			"*.json",
			"vendor/",
			"node_modules/",
		}
	}

	return &DeltaAnalyzer{
		repoPath:       cfg.RepoPath,
		baselineBranch: cfg.BaselineBranch,
		exclusions:     cfg.Exclusions,
	}, nil
}

// Analyze performs delta analysis comparing current state to baseline.
func (a *DeltaAnalyzer) Analyze(ctx context.Context, commitSHA string) (*AnalysisResult, error) {
	// Open git repository
	repo, err := git.PlainOpen(a.repoPath)
	if err != nil {
		return nil, fmt.Errorf("failed to open repository: %w", err)
	}

	// Get baseline commit
	baselineRef, err := repo.Reference(plumbing.NewBranchReferenceName(a.baselineBranch), true)
	if err != nil {
		return nil, fmt.Errorf("failed to get baseline reference: %w", err)
	}

	// Get current commit
	var currentHash plumbing.Hash
	if commitSHA == "" {
		head, err := repo.Head()
		if err != nil {
			return nil, fmt.Errorf("failed to get HEAD: %w", err)
		}
		currentHash = head.Hash()
	} else {
		currentHash = plumbing.NewHash(commitSHA)
	}

	// Calculate deltas per dimension
	vehicleDelta, err := a.analyzeDimension(repo, baselineRef.Hash(), currentHash, "vehicle")
	if err != nil {
		return nil, fmt.Errorf("vehicle dimension analysis failed: %w", err)
	}

	sectorDelta, err := a.analyzeDimension(repo, baselineRef.Hash(), currentHash, "sector")
	if err != nil {
		return nil, fmt.Errorf("sector dimension analysis failed: %w", err)
	}

	platformDelta, err := a.analyzeDimension(repo, baselineRef.Hash(), currentHash, "platform")
	if err != nil {
		return nil, fmt.Errorf("platform dimension analysis failed: %w", err)
	}

	testDelta, err := a.analyzeDimension(repo, baselineRef.Hash(), currentHash, "test")
	if err != nil {
		return nil, fmt.Errorf("test dimension analysis failed: %w", err)
	}

	result := &AnalysisResult{
		CommitSHA:      currentHash.String(),
		BaselineCommit: baselineRef.Hash().String(),
		VehicleDelta:   vehicleDelta,
		SectorDelta:    sectorDelta,
		PlatformDelta:  platformDelta,
		TestDelta:      testDelta,
		TotalFilesChanged: vehicleDelta.FilesChanged + sectorDelta.FilesChanged + 
			platformDelta.FilesChanged + testDelta.FilesChanged,
		Metadata: map[string]interface{}{
			"repo_path":       a.repoPath,
			"baseline_branch": a.baselineBranch,
			"exclusions":      a.exclusions,
		},
	}

	return result, nil
}

// analyzeDimension calculates delta for a specific agnostic dimension.
func (a *DeltaAnalyzer) analyzeDimension(repo *git.Repository, baseHash, currentHash plumbing.Hash, dimension string) (DimensionDelta, error) {
	// Use git diff to calculate changes
	baseCommit, err := repo.CommitObject(baseHash)
	if err != nil {
		return DimensionDelta{}, fmt.Errorf("failed to get base commit: %w", err)
	}

	currentCommit, err := repo.CommitObject(currentHash)
	if err != nil {
		return DimensionDelta{}, fmt.Errorf("failed to get current commit: %w", err)
	}

	baseTree, err := baseCommit.Tree()
	if err != nil {
		return DimensionDelta{}, fmt.Errorf("failed to get base tree: %w", err)
	}

	currentTree, err := currentCommit.Tree()
	if err != nil {
		return DimensionDelta{}, fmt.Errorf("failed to get current tree: %w", err)
	}

	// Get diff between trees
	changes, err := baseTree.Diff(currentTree)
	if err != nil {
		return DimensionDelta{}, fmt.Errorf("failed to diff trees: %w", err)
	}

	// Filter changes by dimension
	dimensionPaths := a.getDimensionPaths(dimension)
	
	var linesAdded, linesDeleted, linesModified, filesChanged int
	
	for _, change := range changes {
		path := change.To.Name
		if change.To.Name == "" {
			path = change.From.Name
		}

		// Check if file belongs to this dimension
		if !a.isInDimension(path, dimensionPaths) {
			continue
		}

		// Check exclusions
		if a.isExcluded(path) {
			continue
		}

		filesChanged++

		// Use git diff --stat to get line counts
		stats, err := a.getFileStats(baseHash.String(), currentHash.String(), path)
		if err != nil {
			continue // Skip files with errors
		}

		linesAdded += stats.Added
		linesDeleted += stats.Deleted
		linesModified += stats.Modified
	}

	// Calculate code delta percentage
	// Simple heuristic: (lines added + deleted) / total baseline lines
	baselineLines, err := a.countBaselineLines(repo, baseHash, dimensionPaths)
	if err != nil {
		baselineLines = 10000 // Fallback estimate
	}

	codeDeltaPct := 0.0
	if baselineLines > 0 {
		codeDeltaPct = (float64(linesAdded+linesDeleted) / float64(baselineLines)) * 100.0
	}

	return DimensionDelta{
		LinesAdded:      linesAdded,
		LinesDeleted:    linesDeleted,
		LinesModified:   linesModified,
		FilesChanged:    filesChanged,
		CodeDeltaPct:    codeDeltaPct,
		ComplexityDelta: 0, // TODO: Implement cyclomatic complexity analysis
	}, nil
}

// getDimensionPaths returns path patterns for a specific dimension.
func (a *DeltaAnalyzer) getDimensionPaths(dimension string) []string {
	switch dimension {
	case "vehicle":
		return []string{
			"services/vehicle-hal/",
			"edge/vehicle-agent/",
			"configs/vehicles/",
		}
	case "sector":
		return []string{
			"services/sector-overlays/",
			"configs/sectors/",
			"configs/odd/",
		}
	case "platform":
		return []string{
			"services/platform-adapters/",
			"infrastructure/",
			"deployment/",
		}
	case "test":
		return []string{
			"testing/",
			"**/*_test.go",
			"**/*.test.ts",
			"**/*.spec.ts",
		}
	default:
		return []string{}
	}
}

// isInDimension checks if a path belongs to the specified dimension.
func (a *DeltaAnalyzer) isInDimension(path string, dimensionPaths []string) bool {
	for _, pattern := range dimensionPaths {
		if strings.HasPrefix(path, strings.TrimSuffix(pattern, "/")) {
			return true
		}
		// Handle wildcard patterns
		if strings.Contains(pattern, "*") {
			matched, _ := filepath.Match(pattern, path)
			if matched {
				return true
			}
		}
	}
	return false
}

// isExcluded checks if a path should be excluded from analysis.
func (a *DeltaAnalyzer) isExcluded(path string) bool {
	for _, exclusion := range a.exclusions {
		if strings.HasPrefix(path, exclusion) {
			return true
		}
		if strings.HasSuffix(exclusion, "*") {
			prefix := strings.TrimSuffix(exclusion, "*")
			if strings.HasPrefix(path, prefix) {
				return true
			}
		}
		matched, _ := filepath.Match(exclusion, filepath.Base(path))
		if matched {
			return true
		}
	}
	return false
}

// FileStats represents line change statistics for a file.
type FileStats struct {
	Added    int
	Deleted  int
	Modified int
}

// getFileStats uses git diff to get line change statistics.
func (a *DeltaAnalyzer) getFileStats(baseCommit, currentCommit, filePath string) (*FileStats, error) {
	cmd := exec.Command("git", "diff", "--numstat", baseCommit, currentCommit, "--", filePath)
	cmd.Dir = a.repoPath
	
	output, err := cmd.Output()
	if err != nil {
		return nil, fmt.Errorf("git diff failed: %w", err)
	}

	// Parse numstat output: "added\tdeleted\tfilename"
	line := strings.TrimSpace(string(output))
	if line == "" {
		return &FileStats{}, nil
	}

	parts := strings.Fields(line)
	if len(parts) < 2 {
		return &FileStats{}, nil
	}

	var added, deleted int
	fmt.Sscanf(parts[0], "%d", &added)
	fmt.Sscanf(parts[1], "%d", &deleted)

	return &FileStats{
		Added:    added,
		Deleted:  deleted,
		Modified: min(added, deleted),
	}, nil
}

// countBaselineLines counts total lines of code in baseline for dimension.
func (a *DeltaAnalyzer) countBaselineLines(repo *git.Repository, baseHash plumbing.Hash, dimensionPaths []string) (int, error) {
	// Simple approximation using cloc or wc
	// For production, integrate with a proper SLOC counter
	return 10000, nil // Placeholder
}

func min(a, b int) int {
	if a < b {
		return a
	}
	return b
}

