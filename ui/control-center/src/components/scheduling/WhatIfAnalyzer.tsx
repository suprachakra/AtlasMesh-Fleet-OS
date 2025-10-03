import React, { useState, useCallback, useEffect, useMemo } from 'react'
import { 
  Play, RotateCcw, TrendingUp, TrendingDown, DollarSign, Clock, MapPin,
  Users, Car, Fuel, AlertTriangle, CheckCircle, BarChart3, PieChart,
  Calendar, Route, Zap, Target, Eye, Download, Copy, Settings, Filter
} from 'lucide-react'
import { Button } from '../ui/Button'
import { Badge } from '../ui/Badge'
import { Card } from '../ui/Card'
import { Dialog, DialogContent, DialogHeader, DialogTitle } from '../ui/Dialog'
import { Input } from '../ui/Input'
import { Select } from '../ui/Select'
import { Alert, AlertDescription } from '../ui/Alert'
import { Progress } from '../ui/Progress'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '../ui/Tabs'

// Types
interface WhatIfScenario {
  id: string
  name: string
  description: string
  baselineId: string
  changes: ScenarioChange[]
  status: 'draft' | 'analyzing' | 'completed' | 'error'
  createdAt: Date
  analysisResults?: AnalysisResults
}

interface ScenarioChange {
  id: string
  type: 'add_trip' | 'modify_trip' | 'remove_trip' | 'reschedule_trip' | 'change_vehicle' | 'change_driver' | 'batch_duplicate'
  targetId?: string
  parameters: Record<string, any>
  description: string
}

interface AnalysisResults {
  id: string
  scenarioId: string
  computedAt: Date
  metrics: {
    eta: ETAAnalysis
    coverage: CoverageAnalysis
    cost: CostAnalysis
    capacity: CapacityAnalysis
    efficiency: EfficiencyAnalysis
    risk: RiskAnalysis
  }
  comparisons: {
    baseline: BaselineComparison
    alternatives: AlternativeComparison[]
  }
  recommendations: Recommendation[]
  constraints: ConstraintViolation[]
}

interface ETAAnalysis {
  averageETA: number
  etaVariance: number
  onTimePerformance: number
  delayedTrips: number
  totalTripTime: number
  criticalPathDelay: number
}

interface CoverageAnalysis {
  totalDistance: number
  uniqueRoutes: number
  geographicCoverage: number
  demandCoverage: number
  serviceGaps: ServiceGap[]
  redundancy: number
}

interface CostAnalysis {
  totalOperatingCost: number
  costPerKm: number
  costPerTrip: number
  fuelCost: number
  laborCost: number
  maintenanceCost: number
  opportunityCost: number
  costEfficiency: number
}

interface CapacityAnalysis {
  vehicleUtilization: number
  driverUtilization: number
  depotUtilization: number
  peakCapacityUsage: number
  bottlenecks: Bottleneck[]
  surplusCapacity: number
}

interface EfficiencyAnalysis {
  routeEfficiency: number
  timeEfficiency: number
  resourceEfficiency: number
  emptyMileage: number
  deadheadRatio: number
  loadFactor: number
}

interface RiskAnalysis {
  overallRiskScore: number
  weatherRisk: number
  trafficRisk: number
  mechanicalRisk: number
  operationalRisk: number
  contingencyNeeded: number
  riskMitigation: string[]
}

interface BaselineComparison {
  etaDelta: number
  costDelta: number
  efficiencyDelta: number
  riskDelta: number
  improvementAreas: string[]
  degradationAreas: string[]
}

interface AlternativeComparison {
  scenarioId: string
  scenarioName: string
  score: number
  advantages: string[]
  disadvantages: string[]
}

interface Recommendation {
  id: string
  type: 'optimization' | 'risk_mitigation' | 'cost_reduction' | 'efficiency_improvement'
  priority: 'low' | 'medium' | 'high' | 'critical'
  title: string
  description: string
  impact: {
    costSaving?: number
    timeReduction?: number
    riskReduction?: number
    efficiencyGain?: number
  }
  implementation: string
  confidence: number
}

interface ConstraintViolation {
  id: string
  constraint: string
  severity: 'minor' | 'major' | 'critical'
  description: string
  affectedTrips: string[]
  suggestedFix: string
}

interface ServiceGap {
  area: string
  demandLevel: number
  currentCoverage: number
  recommendedCoverage: number
}

interface Bottleneck {
  resource: string
  utilizationRate: number
  impactedTrips: number
  suggestedCapacity: number
}

interface WhatIfAnalyzerProps {
  baselineScenario: any
  onScenarioCreated?: (scenario: WhatIfScenario) => void
  onScenarioApplied?: (scenario: WhatIfScenario) => void
  className?: string
}

const WhatIfAnalyzer: React.FC<WhatIfAnalyzerProps> = ({
  baselineScenario,
  onScenarioCreated,
  onScenarioApplied,
  className = ''
}) => {
  // State
  const [scenarios, setScenarios] = useState<WhatIfScenario[]>([])
  const [showCreateDialog, setShowCreateDialog] = useState(false)
  const [showResultsDialog, setShowResultsDialog] = useState(false)
  const [selectedScenario, setSelectedScenario] = useState<WhatIfScenario | null>(null)
  const [activeTab, setActiveTab] = useState('overview')
  const [isAnalyzing, setIsAnalyzing] = useState(false)
  const [newScenario, setNewScenario] = useState({
    name: '',
    description: '',
    changeType: 'add_trip' as const,
    parameters: {}
  })

  // Mock analysis engine
  const runWhatIfAnalysis = useCallback(async (scenario: WhatIfScenario): Promise<AnalysisResults> => {
    // Simulate analysis computation
    await new Promise(resolve => setTimeout(resolve, 3000))

    // Generate mock results based on scenario changes
    const hasAddTrips = scenario.changes.some(c => c.type === 'add_trip')
    const hasReschedule = scenario.changes.some(c => c.type === 'reschedule_trip')
    const hasBatchDuplicate = scenario.changes.some(c => c.type === 'batch_duplicate')

    const results: AnalysisResults = {
      id: `analysis-${Date.now()}`,
      scenarioId: scenario.id,
      computedAt: new Date(),
      metrics: {
        eta: {
          averageETA: hasAddTrips ? 28.5 : 25.2,
          etaVariance: hasReschedule ? 4.2 : 6.1,
          onTimePerformance: hasReschedule ? 94.2 : 87.8,
          delayedTrips: hasAddTrips ? 8 : 5,
          totalTripTime: hasBatchDuplicate ? 1240 : 980,
          criticalPathDelay: hasReschedule ? 12 : 18
        },
        coverage: {
          totalDistance: hasBatchDuplicate ? 2840 : 2240,
          uniqueRoutes: hasAddTrips ? 28 : 24,
          geographicCoverage: hasAddTrips ? 87.5 : 82.1,
          demandCoverage: hasBatchDuplicate ? 96.3 : 89.7,
          serviceGaps: [
            { area: 'Dubai South', demandLevel: 85, currentCoverage: 65, recommendedCoverage: 80 },
            { area: 'Al Barsha', demandLevel: 92, currentCoverage: 78, recommendedCoverage: 90 }
          ],
          redundancy: hasAddTrips ? 1.8 : 1.4
        },
        cost: {
          totalOperatingCost: hasBatchDuplicate ? 18500 : 15200,
          costPerKm: hasReschedule ? 1.85 : 2.12,
          costPerTrip: hasAddTrips ? 42.50 : 38.90,
          fuelCost: hasBatchDuplicate ? 6200 : 4800,
          laborCost: hasAddTrips ? 8900 : 7200,
          maintenanceCost: 2400,
          opportunityCost: hasReschedule ? 800 : 1200,
          costEfficiency: hasReschedule ? 92.1 : 86.4
        },
        capacity: {
          vehicleUtilization: hasBatchDuplicate ? 89.2 : 73.5,
          driverUtilization: hasAddTrips ? 82.1 : 71.8,
          depotUtilization: hasBatchDuplicate ? 94.5 : 78.2,
          peakCapacityUsage: hasBatchDuplicate ? 96.8 : 82.1,
          bottlenecks: [
            { resource: 'Charging Stations', utilizationRate: 94.5, impactedTrips: 12, suggestedCapacity: 25 },
            { resource: 'Maintenance Bay 2', utilizationRate: 87.2, impactedTrips: 6, suggestedCapacity: 3 }
          ],
          surplusCapacity: hasReschedule ? 18.5 : 12.8
        },
        efficiency: {
          routeEfficiency: hasReschedule ? 91.2 : 84.6,
          timeEfficiency: hasReschedule ? 88.9 : 82.1,
          resourceEfficiency: hasBatchDuplicate ? 86.7 : 79.3,
          emptyMileage: hasReschedule ? 8.2 : 12.5,
          deadheadRatio: hasReschedule ? 0.12 : 0.18,
          loadFactor: hasBatchDuplicate ? 0.87 : 0.74
        },
        risk: {
          overallRiskScore: hasReschedule ? 2.1 : 3.4,
          weatherRisk: 1.8,
          trafficRisk: hasAddTrips ? 2.9 : 2.2,
          mechanicalRisk: hasBatchDuplicate ? 3.1 : 2.4,
          operationalRisk: hasReschedule ? 1.9 : 2.8,
          contingencyNeeded: hasAddTrips ? 15 : 12,
          riskMitigation: [
            'Implement dynamic routing for weather conditions',
            'Add backup vehicles for peak hours',
            'Enhanced predictive maintenance scheduling'
          ]
        }
      },
      comparisons: {
        baseline: {
          etaDelta: hasReschedule ? -2.1 : 1.8,
          costDelta: hasBatchDuplicate ? 3200 : hasAddTrips ? 800 : -400,
          efficiencyDelta: hasReschedule ? 6.8 : -2.3,
          riskDelta: hasReschedule ? -1.3 : 0.8,
          improvementAreas: hasReschedule ? 
            ['On-time performance', 'Route efficiency', 'Risk reduction'] :
            hasAddTrips ? ['Geographic coverage', 'Demand satisfaction'] :
            ['Cost efficiency', 'Resource utilization'],
          degradationAreas: hasBatchDuplicate ? 
            ['Operating costs', 'Capacity strain'] :
            hasAddTrips ? ['Cost per trip', 'Complexity'] :
            []
        },
        alternatives: [
          {
            scenarioId: 'alt-001',
            scenarioName: 'Conservative Expansion',
            score: 87.2,
            advantages: ['Lower risk', 'Gradual scaling', 'Cost control'],
            disadvantages: ['Slower coverage growth', 'Limited capacity gains']
          },
          {
            scenarioId: 'alt-002',
            scenarioName: 'Aggressive Optimization',
            score: 91.8,
            advantages: ['Maximum efficiency', 'Best ROI', 'Optimal resource use'],
            disadvantages: ['Higher complexity', 'Less flexibility', 'Weather sensitivity']
          }
        ]
      },
      recommendations: [
        {
          id: 'rec-001',
          type: 'optimization',
          priority: 'high',
          title: 'Implement Dynamic Route Optimization',
          description: 'Deploy real-time route optimization to reduce travel time by 12% and improve fuel efficiency',
          impact: {
            timeReduction: 18,
            costSaving: 2400,
            efficiencyGain: 12
          },
          implementation: 'Integrate with traffic management systems and weather APIs',
          confidence: 89
        },
        {
          id: 'rec-002',
          type: 'cost_reduction',
          priority: 'medium',
          title: 'Optimize Charging Schedule',
          description: 'Shift charging to off-peak hours to reduce energy costs by 15%',
          impact: {
            costSaving: 1800,
            efficiencyGain: 8
          },
          implementation: 'Update charging management system with time-of-use pricing',
          confidence: 94
        },
        {
          id: 'rec-003',
          type: 'risk_mitigation',
          priority: 'high',
          title: 'Add Weather Contingency Routes',
          description: 'Develop alternative routes for adverse weather conditions',
          impact: {
            riskReduction: 25,
            timeReduction: 8
          },
          implementation: 'Map weather-safe routes and update navigation systems',
          confidence: 82
        }
      ],
      constraints: scenario.changes.some(c => c.type === 'batch_duplicate') ? [
        {
          id: 'const-001',
          constraint: 'Depot Capacity',
          severity: 'major',
          description: 'Peak hour vehicle count exceeds depot capacity by 8 vehicles',
          affectedTrips: ['trip-045', 'trip-046', 'trip-047'],
          suggestedFix: 'Stagger departure times or add temporary parking capacity'
        },
        {
          id: 'const-002',
          constraint: 'Driver Availability',
          severity: 'minor',
          description: 'Two drivers would exceed weekly hour limits',
          affectedTrips: ['trip-052', 'trip-058'],
          suggestedFix: 'Assign backup drivers or reduce trip duration'
        }
      ] : []
    }

    return results
  }, [])

  // Create and analyze scenario
  const createScenario = useCallback(async () => {
    const scenario: WhatIfScenario = {
      id: `scenario-${Date.now()}`,
      name: newScenario.name,
      description: newScenario.description,
      baselineId: baselineScenario.id,
      changes: [
        {
          id: `change-${Date.now()}`,
          type: newScenario.changeType,
          parameters: newScenario.parameters,
          description: `${newScenario.changeType.replace('_', ' ')} operation`
        }
      ],
      status: 'analyzing',
      createdAt: new Date()
    }

    setScenarios(prev => [scenario, ...prev])
    setShowCreateDialog(false)
    setIsAnalyzing(true)
    onScenarioCreated?.(scenario)

    try {
      const results = await runWhatIfAnalysis(scenario)
      
      setScenarios(prev => prev.map(s => 
        s.id === scenario.id 
          ? { ...s, status: 'completed' as const, analysisResults: results }
          : s
      ))
    } catch (error) {
      setScenarios(prev => prev.map(s => 
        s.id === scenario.id 
          ? { ...s, status: 'error' as const }
          : s
      ))
    } finally {
      setIsAnalyzing(false)
    }

    // Reset form
    setNewScenario({
      name: '',
      description: '',
      changeType: 'add_trip',
      parameters: {}
    })
  }, [newScenario, baselineScenario.id, onScenarioCreated, runWhatIfAnalysis])

  const handleViewResults = useCallback((scenario: WhatIfScenario) => {
    setSelectedScenario(scenario)
    setShowResultsDialog(true)
  }, [])

  const handleApplyScenario = useCallback((scenario: WhatIfScenario) => {
    onScenarioApplied?.(scenario)
    console.info('Scenario applied', {
      scenarioId: scenario.id,
      scenarioName: scenario.name,
      changes: scenario.changes.length,
      timestamp: new Date().toISOString()
    })
  }, [onScenarioApplied])

  const getStatusColor = (status: WhatIfScenario['status']) => {
    switch (status) {
      case 'completed': return 'bg-green-100 text-green-800'
      case 'analyzing': return 'bg-blue-100 text-blue-800'
      case 'error': return 'bg-red-100 text-red-800'
      case 'draft': return 'bg-gray-100 text-gray-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getImpactColor = (value: number, isInverted = false) => {
    const positive = isInverted ? value < 0 : value > 0
    return positive ? 'text-green-600' : value < 0 ? 'text-red-600' : 'text-gray-600'
  }

  const formatDelta = (value: number, unit = '', showSign = true) => {
    const sign = showSign && value > 0 ? '+' : ''
    return `${sign}${value.toFixed(1)}${unit}`
  }

  return (
    <div className={`space-y-6 ${className}`}>
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex items-center space-x-3">
          <Target className="w-6 h-6 text-purple-600" />
          <h2 className="text-2xl font-semibold text-gray-900">What-If Analysis</h2>
          {isAnalyzing && (
            <Badge className="bg-blue-100 text-blue-800">
              <BarChart3 className="w-3 h-3 mr-1 animate-pulse" />
              Analyzing...
            </Badge>
          )}
        </div>
        <Button onClick={() => setShowCreateDialog(true)}>
          <Play className="w-4 h-4 mr-2" />
          Create Scenario
        </Button>
      </div>

      {/* Baseline Summary */}
      <Card className="p-6 border-blue-200 bg-blue-50">
        <div className="flex items-center justify-between mb-4">
          <h3 className="text-lg font-medium text-blue-900">Current Baseline</h3>
          <Badge className="bg-blue-100 text-blue-800">Active</Badge>
        </div>
        <div className="grid grid-cols-4 gap-6 text-sm">
          <div>
            <div className="text-blue-700">Total Trips</div>
            <div className="text-2xl font-bold text-blue-900">42</div>
          </div>
          <div>
            <div className="text-blue-700">Operating Cost</div>
            <div className="text-2xl font-bold text-blue-900">$15,200</div>
          </div>
          <div>
            <div className="text-blue-700">Avg ETA</div>
            <div className="text-2xl font-bold text-blue-900">25.2min</div>
          </div>
          <div>
            <div className="text-blue-700">Efficiency</div>
            <div className="text-2xl font-bold text-blue-900">84.6%</div>
          </div>
        </div>
      </Card>

      {/* Scenarios List */}
      <div className="space-y-4">
        <h3 className="text-lg font-medium text-gray-900">Analysis Scenarios</h3>
        
        {scenarios.map(scenario => (
          <Card key={scenario.id} className="p-6 hover:shadow-md transition-shadow">
            <div className="flex items-start justify-between">
              <div className="flex-1">
                <div className="flex items-center space-x-3 mb-3">
                  <h4 className="text-lg font-medium text-gray-900">{scenario.name}</h4>
                  <Badge className={getStatusColor(scenario.status)}>
                    {scenario.status}
                  </Badge>
                  <Badge variant="outline">
                    {scenario.changes.length} change(s)
                  </Badge>
                </div>
                
                <p className="text-gray-600 mb-4">{scenario.description}</p>
                
                <div className="flex items-center space-x-4 text-sm text-gray-600 mb-4">
                  <div className="flex items-center space-x-1">
                    <Calendar className="w-4 h-4" />
                    <span>Created: {scenario.createdAt.toLocaleDateString()}</span>
                  </div>
                  <div className="flex items-center space-x-1">
                    <Settings className="w-4 h-4" />
                    <span>Changes: {scenario.changes.map(c => c.type.replace('_', ' ')).join(', ')}</span>
                  </div>
                </div>

                {/* Results Preview */}
                {scenario.analysisResults && (
                  <div className="grid grid-cols-4 gap-4 p-4 bg-gray-50 rounded-lg">
                    <div className="text-center">
                      <div className={`text-lg font-bold ${getImpactColor(scenario.analysisResults.comparisons.baseline.etaDelta, true)}`}>
                        {formatDelta(scenario.analysisResults.comparisons.baseline.etaDelta, 'min')}
                      </div>
                      <div className="text-xs text-gray-600">ETA Change</div>
                    </div>
                    <div className="text-center">
                      <div className={`text-lg font-bold ${getImpactColor(scenario.analysisResults.comparisons.baseline.costDelta, true)}`}>
                        {formatDelta(scenario.analysisResults.comparisons.baseline.costDelta, '', false)}
                      </div>
                      <div className="text-xs text-gray-600">Cost Impact</div>
                    </div>
                    <div className="text-center">
                      <div className={`text-lg font-bold ${getImpactColor(scenario.analysisResults.comparisons.baseline.efficiencyDelta)}`}>
                        {formatDelta(scenario.analysisResults.comparisons.baseline.efficiencyDelta, '%')}
                      </div>
                      <div className="text-xs text-gray-600">Efficiency</div>
                    </div>
                    <div className="text-center">
                      <div className={`text-lg font-bold ${getImpactColor(scenario.analysisResults.comparisons.baseline.riskDelta, true)}`}>
                        {formatDelta(scenario.analysisResults.comparisons.baseline.riskDelta)}
                      </div>
                      <div className="text-xs text-gray-600">Risk Score</div>
                    </div>
                  </div>
                )}
              </div>
              
              <div className="flex items-center space-x-2">
                {scenario.status === 'completed' && (
                  <>
                    <Button
                      variant="outline"
                      size="sm"
                      onClick={() => handleViewResults(scenario)}
                    >
                      <Eye className="w-4 h-4" />
                    </Button>
                    <Button
                      size="sm"
                      onClick={() => handleApplyScenario(scenario)}
                    >
                      Apply
                    </Button>
                  </>
                )}
                
                {scenario.status === 'analyzing' && (
                  <div className="flex items-center space-x-2 text-blue-600">
                    <BarChart3 className="w-4 h-4 animate-pulse" />
                    <span className="text-sm">Analyzing...</span>
                  </div>
                )}
              </div>
            </div>
          </Card>
        ))}
        
        {scenarios.length === 0 && (
          <div className="text-center py-12">
            <Target className="w-16 h-16 text-gray-400 mx-auto mb-4" />
            <h3 className="text-lg font-medium text-gray-900 mb-2">No Scenarios Created</h3>
            <p className="text-gray-600">Create your first what-if scenario to analyze potential changes</p>
          </div>
        )}
      </div>

      {/* Create Scenario Dialog */}
      <Dialog open={showCreateDialog} onOpenChange={setShowCreateDialog}>
        <DialogContent className="max-w-2xl">
          <DialogHeader>
            <DialogTitle>Create What-If Scenario</DialogTitle>
          </DialogHeader>
          
          <div className="space-y-4">
            <div>
              <label className="block text-sm font-medium text-gray-700 mb-2">
                Scenario Name *
              </label>
              <Input
                value={newScenario.name}
                onChange={(e) => setNewScenario(prev => ({ ...prev, name: e.target.value }))}
                placeholder="e.g., Add Evening Routes, Optimize Peak Hours"
                required
              />
            </div>
            
            <div>
              <label className="block text-sm font-medium text-gray-700 mb-2">
                Description
              </label>
              <textarea
                value={newScenario.description}
                onChange={(e) => setNewScenario(prev => ({ ...prev, description: e.target.value }))}
                placeholder="Describe what changes you want to analyze..."
                className="w-full p-3 border border-gray-300 rounded-md"
                rows={3}
              />
            </div>
            
            <div>
              <label className="block text-sm font-medium text-gray-700 mb-2">
                Change Type
              </label>
              <Select
                value={newScenario.changeType}
                onValueChange={(value) => setNewScenario(prev => ({ 
                  ...prev, 
                  changeType: value as any
                }))}
              >
                <option value="add_trip">Add New Trip</option>
                <option value="modify_trip">Modify Existing Trip</option>
                <option value="remove_trip">Remove Trip</option>
                <option value="reschedule_trip">Reschedule Trip</option>
                <option value="change_vehicle">Change Vehicle Assignment</option>
                <option value="change_driver">Change Driver Assignment</option>
                <option value="batch_duplicate">Batch Duplicate Trips</option>
              </Select>
            </div>
            
            {/* Change-specific parameters */}
            {newScenario.changeType === 'add_trip' && (
              <div className="p-4 bg-blue-50 rounded-lg">
                <h4 className="font-medium text-blue-900 mb-2">Trip Parameters</h4>
                <div className="grid grid-cols-2 gap-3 text-sm">
                  <Input placeholder="Origin" />
                  <Input placeholder="Destination" />
                  <Input placeholder="Departure Time" type="time" />
                  <Input placeholder="Vehicle Type" />
                </div>
              </div>
            )}
            
            {newScenario.changeType === 'batch_duplicate' && (
              <div className="p-4 bg-purple-50 rounded-lg">
                <h4 className="font-medium text-purple-900 mb-2">Batch Parameters</h4>
                <div className="grid grid-cols-2 gap-3 text-sm">
                  <Input placeholder="Number of Copies" type="number" />
                  <Input placeholder="Time Offset (minutes)" type="number" />
                  <Select>
                    <option value="">Select Route Pattern</option>
                    <option value="peak_hours">Peak Hours Routes</option>
                    <option value="off_peak">Off-Peak Routes</option>
                    <option value="weekend">Weekend Routes</option>
                  </Select>
                  <div className="flex items-center space-x-2">
                    <input type="checkbox" id="auto_assign" />
                    <label htmlFor="auto_assign" className="text-sm">Auto-assign vehicles</label>
                  </div>
                </div>
              </div>
            )}
            
            <Alert>
              <Target className="w-4 h-4" />
              <AlertDescription>
                Analysis will compare this scenario against the current baseline and provide 
                detailed metrics on ETA, coverage, cost impact, and operational efficiency.
              </AlertDescription>
            </Alert>
            
            <div className="flex justify-end space-x-3 pt-4">
              <Button
                variant="outline"
                onClick={() => setShowCreateDialog(false)}
              >
                Cancel
              </Button>
              <Button
                onClick={createScenario}
                disabled={!newScenario.name}
              >
                <Play className="w-4 h-4 mr-2" />
                Create & Analyze
              </Button>
            </div>
          </div>
        </DialogContent>
      </Dialog>

      {/* Results Dialog */}
      <Dialog open={showResultsDialog} onOpenChange={setShowResultsDialog}>
        <DialogContent className="max-w-7xl max-h-[90vh] overflow-y-auto">
          <DialogHeader>
            <DialogTitle>Analysis Results: {selectedScenario?.name}</DialogTitle>
          </DialogHeader>

          {selectedScenario?.analysisResults && (
            <Tabs value={activeTab} onValueChange={setActiveTab}>
              <TabsList>
                <TabsTrigger value="overview">Overview</TabsTrigger>
                <TabsTrigger value="metrics">Detailed Metrics</TabsTrigger>
                <TabsTrigger value="comparisons">Comparisons</TabsTrigger>
                <TabsTrigger value="recommendations">Recommendations</TabsTrigger>
                <TabsTrigger value="constraints">Constraints</TabsTrigger>
              </TabsList>

              <TabsContent value="overview" className="space-y-6">
                {/* Key Metrics Summary */}
                <div className="grid grid-cols-4 gap-6">
                  <Card className="p-4 text-center">
                    <div className="flex items-center justify-center mb-2">
                      <Clock className="w-5 h-5 text-blue-600 mr-2" />
                      <span className="font-medium">ETA Impact</span>
                    </div>
                    <div className={`text-2xl font-bold ${getImpactColor(selectedScenario.analysisResults.comparisons.baseline.etaDelta, true)}`}>
                      {formatDelta(selectedScenario.analysisResults.comparisons.baseline.etaDelta, 'min')}
                    </div>
                    <div className="text-sm text-gray-600">vs baseline</div>
                  </Card>

                  <Card className="p-4 text-center">
                    <div className="flex items-center justify-center mb-2">
                      <DollarSign className="w-5 h-5 text-green-600 mr-2" />
                      <span className="font-medium">Cost Impact</span>
                    </div>
                    <div className={`text-2xl font-bold ${getImpactColor(selectedScenario.analysisResults.comparisons.baseline.costDelta, true)}`}>
                      ${Math.abs(selectedScenario.analysisResults.comparisons.baseline.costDelta).toLocaleString()}
                    </div>
                    <div className="text-sm text-gray-600">
                      {selectedScenario.analysisResults.comparisons.baseline.costDelta > 0 ? 'increase' : 'savings'}
                    </div>
                  </Card>

                  <Card className="p-4 text-center">
                    <div className="flex items-center justify-center mb-2">
                      <TrendingUp className="w-5 h-5 text-purple-600 mr-2" />
                      <span className="font-medium">Efficiency</span>
                    </div>
                    <div className={`text-2xl font-bold ${getImpactColor(selectedScenario.analysisResults.comparisons.baseline.efficiencyDelta)}`}>
                      {formatDelta(selectedScenario.analysisResults.comparisons.baseline.efficiencyDelta, '%')}
                    </div>
                    <div className="text-sm text-gray-600">efficiency change</div>
                  </Card>

                  <Card className="p-4 text-center">
                    <div className="flex items-center justify-center mb-2">
                      <AlertTriangle className="w-5 h-5 text-orange-600 mr-2" />
                      <span className="font-medium">Risk Score</span>
                    </div>
                    <div className={`text-2xl font-bold ${getImpactColor(selectedScenario.analysisResults.comparisons.baseline.riskDelta, true)}`}>
                      {selectedScenario.analysisResults.metrics.risk.overallRiskScore.toFixed(1)}
                    </div>
                    <div className="text-sm text-gray-600">
                      {formatDelta(selectedScenario.analysisResults.comparisons.baseline.riskDelta)} vs baseline
                    </div>
                  </Card>
                </div>

                {/* Improvement Areas */}
                <div className="grid grid-cols-2 gap-6">
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-green-900 mb-3 flex items-center">
                      <CheckCircle className="w-5 h-5 mr-2" />
                      Improvements
                    </h3>
                    <div className="space-y-2">
                      {selectedScenario.analysisResults.comparisons.baseline.improvementAreas.map((area, index) => (
                        <div key={index} className="flex items-center space-x-2">
                          <TrendingUp className="w-4 h-4 text-green-600" />
                          <span className="text-sm text-green-800">{area}</span>
                        </div>
                      ))}
                    </div>
                  </Card>

                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-red-900 mb-3 flex items-center">
                      <AlertTriangle className="w-5 h-5 mr-2" />
                      Trade-offs
                    </h3>
                    <div className="space-y-2">
                      {selectedScenario.analysisResults.comparisons.baseline.degradationAreas.length > 0 ? (
                        selectedScenario.analysisResults.comparisons.baseline.degradationAreas.map((area, index) => (
                          <div key={index} className="flex items-center space-x-2">
                            <TrendingDown className="w-4 h-4 text-red-600" />
                            <span className="text-sm text-red-800">{area}</span>
                          </div>
                        ))
                      ) : (
                        <div className="flex items-center space-x-2">
                          <CheckCircle className="w-4 h-4 text-green-600" />
                          <span className="text-sm text-green-800">No significant trade-offs identified</span>
                        </div>
                      )}
                    </div>
                  </Card>
                </div>
              </TabsContent>

              <TabsContent value="metrics" className="space-y-6">
                <div className="grid grid-cols-2 gap-6">
                  {/* ETA Metrics */}
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">ETA & Performance</h3>
                    <div className="space-y-3 text-sm">
                      <div className="flex justify-between">
                        <span>Average ETA:</span>
                        <span className="font-medium">{selectedScenario.analysisResults.metrics.eta.averageETA.toFixed(1)} min</span>
                      </div>
                      <div className="flex justify-between">
                        <span>On-time Performance:</span>
                        <span className="font-medium">{selectedScenario.analysisResults.metrics.eta.onTimePerformance.toFixed(1)}%</span>
                      </div>
                      <div className="flex justify-between">
                        <span>Delayed Trips:</span>
                        <span className="font-medium">{selectedScenario.analysisResults.metrics.eta.delayedTrips}</span>
                      </div>
                      <div className="flex justify-between">
                        <span>Total Trip Time:</span>
                        <span className="font-medium">{selectedScenario.analysisResults.metrics.eta.totalTripTime} min</span>
                      </div>
                    </div>
                  </Card>

                  {/* Cost Metrics */}
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Cost Analysis</h3>
                    <div className="space-y-3 text-sm">
                      <div className="flex justify-between">
                        <span>Total Operating Cost:</span>
                        <span className="font-medium">${selectedScenario.analysisResults.metrics.cost.totalOperatingCost.toLocaleString()}</span>
                      </div>
                      <div className="flex justify-between">
                        <span>Cost per Trip:</span>
                        <span className="font-medium">${selectedScenario.analysisResults.metrics.cost.costPerTrip.toFixed(2)}</span>
                      </div>
                      <div className="flex justify-between">
                        <span>Cost per KM:</span>
                        <span className="font-medium">${selectedScenario.analysisResults.metrics.cost.costPerKm.toFixed(2)}</span>
                      </div>
                      <div className="flex justify-between">
                        <span>Cost Efficiency:</span>
                        <span className="font-medium">{selectedScenario.analysisResults.metrics.cost.costEfficiency.toFixed(1)}%</span>
                      </div>
                    </div>
                  </Card>

                  {/* Coverage Metrics */}
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Coverage Analysis</h3>
                    <div className="space-y-3 text-sm">
                      <div className="flex justify-between">
                        <span>Total Distance:</span>
                        <span className="font-medium">{selectedScenario.analysisResults.metrics.coverage.totalDistance.toLocaleString()} km</span>
                      </div>
                      <div className="flex justify-between">
                        <span>Unique Routes:</span>
                        <span className="font-medium">{selectedScenario.analysisResults.metrics.coverage.uniqueRoutes}</span>
                      </div>
                      <div className="flex justify-between">
                        <span>Geographic Coverage:</span>
                        <span className="font-medium">{selectedScenario.analysisResults.metrics.coverage.geographicCoverage.toFixed(1)}%</span>
                      </div>
                      <div className="flex justify-between">
                        <span>Demand Coverage:</span>
                        <span className="font-medium">{selectedScenario.analysisResults.metrics.coverage.demandCoverage.toFixed(1)}%</span>
                      </div>
                    </div>
                  </Card>

                  {/* Capacity Metrics */}
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Capacity Utilization</h3>
                    <div className="space-y-3 text-sm">
                      <div className="flex justify-between items-center">
                        <span>Vehicle Utilization:</span>
                        <div className="flex items-center space-x-2">
                          <Progress value={selectedScenario.analysisResults.metrics.capacity.vehicleUtilization} className="w-16 h-2" />
                          <span className="font-medium">{selectedScenario.analysisResults.metrics.capacity.vehicleUtilization.toFixed(1)}%</span>
                        </div>
                      </div>
                      <div className="flex justify-between items-center">
                        <span>Driver Utilization:</span>
                        <div className="flex items-center space-x-2">
                          <Progress value={selectedScenario.analysisResults.metrics.capacity.driverUtilization} className="w-16 h-2" />
                          <span className="font-medium">{selectedScenario.analysisResults.metrics.capacity.driverUtilization.toFixed(1)}%</span>
                        </div>
                      </div>
                      <div className="flex justify-between items-center">
                        <span>Depot Utilization:</span>
                        <div className="flex items-center space-x-2">
                          <Progress value={selectedScenario.analysisResults.metrics.capacity.depotUtilization} className="w-16 h-2" />
                          <span className="font-medium">{selectedScenario.analysisResults.metrics.capacity.depotUtilization.toFixed(1)}%</span>
                        </div>
                      </div>
                    </div>
                  </Card>
                </div>
              </TabsContent>

              <TabsContent value="recommendations" className="space-y-4">
                <div className="space-y-4">
                  {selectedScenario.analysisResults.recommendations.map(rec => (
                    <Card key={rec.id} className="p-4">
                      <div className="flex items-start justify-between mb-3">
                        <div className="flex-1">
                          <div className="flex items-center space-x-3 mb-2">
                            <h4 className="font-medium text-gray-900">{rec.title}</h4>
                            <Badge className={
                              rec.priority === 'critical' ? 'bg-red-100 text-red-800' :
                              rec.priority === 'high' ? 'bg-orange-100 text-orange-800' :
                              rec.priority === 'medium' ? 'bg-yellow-100 text-yellow-800' :
                              'bg-blue-100 text-blue-800'
                            }>
                              {rec.priority} priority
                            </Badge>
                            <Badge variant="outline" className="capitalize">
                              {rec.type.replace('_', ' ')}
                            </Badge>
                          </div>
                          <p className="text-gray-600 text-sm mb-3">{rec.description}</p>
                          <p className="text-gray-500 text-sm mb-3">{rec.implementation}</p>
                        </div>
                        <div className="text-right">
                          <div className="text-sm text-gray-600 mb-1">Confidence</div>
                          <div className="text-lg font-bold text-blue-600">{rec.confidence}%</div>
                        </div>
                      </div>

                      <div className="grid grid-cols-4 gap-4 text-sm">
                        {rec.impact.costSaving && (
                          <div className="text-center p-2 bg-green-50 rounded">
                            <div className="text-green-600 font-medium">${rec.impact.costSaving.toLocaleString()}</div>
                            <div className="text-green-700 text-xs">Cost Savings</div>
                          </div>
                        )}
                        {rec.impact.timeReduction && (
                          <div className="text-center p-2 bg-blue-50 rounded">
                            <div className="text-blue-600 font-medium">{rec.impact.timeReduction}min</div>
                            <div className="text-blue-700 text-xs">Time Reduction</div>
                          </div>
                        )}
                        {rec.impact.efficiencyGain && (
                          <div className="text-center p-2 bg-purple-50 rounded">
                            <div className="text-purple-600 font-medium">{rec.impact.efficiencyGain}%</div>
                            <div className="text-purple-700 text-xs">Efficiency Gain</div>
                          </div>
                        )}
                        {rec.impact.riskReduction && (
                          <div className="text-center p-2 bg-orange-50 rounded">
                            <div className="text-orange-600 font-medium">{rec.impact.riskReduction}%</div>
                            <div className="text-orange-700 text-xs">Risk Reduction</div>
                          </div>
                        )}
                      </div>
                    </Card>
                  ))}
                </div>
              </TabsContent>

              <TabsContent value="constraints" className="space-y-4">
                {selectedScenario.analysisResults.constraints.length > 0 ? (
                  <div className="space-y-4">
                    {selectedScenario.analysisResults.constraints.map(constraint => (
                      <Card key={constraint.id} className="p-4 border-l-4 border-red-400">
                        <div className="flex items-start justify-between mb-3">
                          <div className="flex-1">
                            <div className="flex items-center space-x-3 mb-2">
                              <h4 className="font-medium text-gray-900">{constraint.constraint}</h4>
                              <Badge className={
                                constraint.severity === 'critical' ? 'bg-red-100 text-red-800' :
                                constraint.severity === 'major' ? 'bg-orange-100 text-orange-800' :
                                'bg-yellow-100 text-yellow-800'
                              }>
                                {constraint.severity}
                              </Badge>
                            </div>
                            <p className="text-gray-600 text-sm mb-3">{constraint.description}</p>
                            <div className="p-2 bg-blue-50 rounded border border-blue-200">
                              <div className="text-sm font-medium text-blue-900">Suggested Fix:</div>
                              <div className="text-sm text-blue-700">{constraint.suggestedFix}</div>
                            </div>
                          </div>
                        </div>
                        <div className="text-sm text-gray-600">
                          Affects {constraint.affectedTrips.length} trip(s): {constraint.affectedTrips.join(', ')}
                        </div>
                      </Card>
                    ))}
                  </div>
                ) : (
                  <div className="text-center py-12">
                    <CheckCircle className="w-16 h-16 text-green-500 mx-auto mb-4" />
                    <h3 className="text-lg font-medium text-gray-900 mb-2">No Constraint Violations</h3>
                    <p className="text-gray-600">This scenario operates within all defined constraints</p>
                  </div>
                )}
              </TabsContent>
            </Tabs>
          )}

          <div className="flex justify-between pt-4 border-t">
            <div className="flex space-x-2">
              <Button variant="outline" size="sm">
                <Download className="w-4 h-4 mr-2" />
                Export Results
              </Button>
              <Button variant="outline" size="sm">
                <Copy className="w-4 h-4 mr-2" />
                Duplicate Scenario
              </Button>
            </div>
            <div className="flex space-x-2">
              <Button variant="outline" onClick={() => setShowResultsDialog(false)}>
                Close
              </Button>
              {selectedScenario && (
                <Button onClick={() => {
                  handleApplyScenario(selectedScenario)
                  setShowResultsDialog(false)
                }}>
                  Apply Scenario
                </Button>
              )}
            </div>
          </div>
        </DialogContent>
      </Dialog>
    </div>
  )
}

export default WhatIfAnalyzer
