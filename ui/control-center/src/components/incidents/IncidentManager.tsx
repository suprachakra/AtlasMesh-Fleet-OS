import React, { useState, useCallback } from 'react'
import { 
  AlertTriangle, FileText, Camera, Download, ExternalLink, Clock,
  User, MapPin, Tag, Shield, Archive, Eye, Edit, Trash2, Plus
} from 'lucide-react'
import { Button } from '../ui/Button'
import { Badge } from '../ui/Badge'
import { Card } from '../ui/Card'
import { Dialog, DialogContent, DialogHeader, DialogTitle } from '../ui/Dialog'
import { Input } from '../ui/Input'
import { Textarea } from '../ui/Textarea'
import { Select } from '../ui/Select'
import { Alert, AlertDescription } from '../ui/Alert'

// Types
interface Incident {
  id: string
  title: string
  description: string
  severity: 'low' | 'medium' | 'high' | 'critical'
  status: 'open' | 'investigating' | 'resolved' | 'closed'
  type: 'safety' | 'operational' | 'technical' | 'compliance'
  vehicleId?: string
  vehicleName?: string
  location?: { lat: number; lng: number; address: string }
  reportedBy: string
  assignedTo?: string
  createdAt: Date
  updatedAt: Date
  resolvedAt?: Date
  tags: string[]
  evidence: Evidence[]
  timeline: IncidentEvent[]
  relatedIncidents: string[]
}

interface Evidence {
  id: string
  type: 'photo' | 'video' | 'document' | 'log' | 'telemetry'
  filename: string
  url: string
  description?: string
  capturedAt: Date
  capturedBy: string
  hash: string // For integrity verification
  size: number
}

interface IncidentEvent {
  id: string
  type: 'created' | 'updated' | 'assigned' | 'evidence_added' | 'status_changed' | 'comment'
  description: string
  timestamp: Date
  user: string
  metadata?: Record<string, unknown>
}

interface IncidentManagerProps {
  vehicleId?: string
  vehicleName?: string
  location?: { lat: number; lng: number; address?: string }
  onIncidentCreated?: (incident: Incident) => void
  className?: string
}

const IncidentManager: React.FC<IncidentManagerProps> = ({
  vehicleId,
  vehicleName,
  location,
  onIncidentCreated,
  className = ''
}) => {
  // State
  const [showCreateDialog, setShowCreateDialog] = useState(false)
  const [showViewDialog, setShowViewDialog] = useState(false)
  const [selectedIncident, setSelectedIncident] = useState<Incident | null>(null)
  const [incidents, setIncidents] = useState<Incident[]>([])
  const [isLoading, setIsLoading] = useState(false)

  // Form state
  const [newIncident, setNewIncident] = useState({
    title: '',
    description: '',
    severity: 'medium' as const,
    type: 'operational' as const,
    tags: ''
  })

  // Mock data for demonstration
  const mockIncidents: Incident[] = [
    {
      id: 'inc-001',
      title: 'Vehicle Emergency Stop Triggered',
      description: 'Vehicle Atlas-001 executed emergency stop due to obstacle detection failure',
      severity: 'critical',
      status: 'investigating',
      type: 'safety',
      vehicleId: 'v1',
      vehicleName: 'Atlas-001',
      location: { lat: 25.2048, lng: 55.2708, address: 'Dubai Mall Parking' },
      reportedBy: 'System Automated',
      assignedTo: 'Ahmed Al-Mansouri',
      createdAt: new Date(Date.now() - 2 * 60 * 60 * 1000),
      updatedAt: new Date(Date.now() - 30 * 60 * 1000),
      tags: ['emergency-stop', 'obstacle-detection', 'safety-critical'],
      evidence: [
        {
          id: 'ev-001',
          type: 'photo',
          filename: 'scene_photo_001.jpg',
          url: '/evidence/scene_photo_001.jpg',
          description: 'Scene photo at time of incident',
          capturedAt: new Date(Date.now() - 2 * 60 * 60 * 1000),
          capturedBy: 'Vehicle Camera System',
          hash: 'sha256:a1b2c3d4e5f6...',
          size: 2048576
        },
        {
          id: 'ev-002',
          type: 'log',
          filename: 'system_logs_001.json',
          url: '/evidence/system_logs_001.json',
          description: 'System logs from 30 seconds before incident',
          capturedAt: new Date(Date.now() - 2 * 60 * 60 * 1000),
          capturedBy: 'Log Collector',
          hash: 'sha256:f6e5d4c3b2a1...',
          size: 1024768
        }
      ],
      timeline: [
        {
          id: 'evt-001',
          type: 'created',
          description: 'Incident automatically created by safety monitoring system',
          timestamp: new Date(Date.now() - 2 * 60 * 60 * 1000),
          user: 'System'
        },
        {
          id: 'evt-002',
          type: 'assigned',
          description: 'Incident assigned to Ahmed Al-Mansouri for investigation',
          timestamp: new Date(Date.now() - 90 * 60 * 1000),
          user: 'Operations Manager'
        },
        {
          id: 'evt-003',
          type: 'evidence_added',
          description: 'Scene photo and system logs automatically collected',
          timestamp: new Date(Date.now() - 90 * 60 * 1000),
          user: 'Evidence Collector'
        }
      ],
      relatedIncidents: []
    }
  ]

  // Handlers
  const handleCreateIncident = useCallback(async () => {
    setIsLoading(true)
    
    try {
      const incident: Incident = {
        id: `inc-${Date.now()}`,
        title: newIncident.title,
        description: newIncident.description,
        severity: newIncident.severity,
        status: 'open',
        type: newIncident.type,
        vehicleId,
        vehicleName,
        location,
        reportedBy: 'Current User', // TODO: Get from auth context
        createdAt: new Date(),
        updatedAt: new Date(),
        tags: newIncident.tags.split(',').map(t => t.trim()).filter(Boolean),
        evidence: [],
        timeline: [
          {
            id: `evt-${Date.now()}`,
            type: 'created',
            description: 'Incident created manually',
            timestamp: new Date(),
            user: 'Current User'
          }
        ],
        relatedIncidents: []
      }

      // Simulate API call
      await new Promise(resolve => setTimeout(resolve, 1000))
      
      setIncidents(prev => [incident, ...prev])
      onIncidentCreated?.(incident)
      
      // Reset form
      setNewIncident({
        title: '',
        description: '',
        severity: 'medium',
        type: 'operational',
        tags: ''
      })
      
      setShowCreateDialog(false)
      
      console.info('Incident created', {
        incidentId: incident.id,
        vehicleId,
        severity: incident.severity,
        timestamp: new Date().toISOString()
      })
    } catch (error) {
      console.error('Failed to create incident:', error)
    } finally {
      setIsLoading(false)
    }
  }, [newIncident, vehicleId, vehicleName, location, onIncidentCreated])

  const handleViewIncident = useCallback((incident: Incident) => {
    setSelectedIncident(incident)
    setShowViewDialog(true)
  }, [])

  const handleExportEvidence = useCallback((incident: Incident) => {
    // Create evidence export package
    const exportData = {
      incident: {
        id: incident.id,
        title: incident.title,
        description: incident.description,
        severity: incident.severity,
        vehicleId: incident.vehicleId,
        location: incident.location,
        createdAt: incident.createdAt,
        timeline: incident.timeline
      },
      evidence: incident.evidence.map(ev => ({
        id: ev.id,
        type: ev.type,
        filename: ev.filename,
        description: ev.description,
        capturedAt: ev.capturedAt,
        hash: ev.hash,
        size: ev.size
      })),
      exportedAt: new Date(),
      exportedBy: 'Current User',
      signature: 'digital_signature_placeholder'
    }

    // In real implementation, this would generate a signed export package
    const blob = new Blob([JSON.stringify(exportData, null, 2)], { type: 'application/json' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = `incident_${incident.id}_evidence_export.json`
    document.body.appendChild(a)
    a.click()
    document.body.removeChild(a)
    URL.revokeObjectURL(url)

    console.info('Evidence exported', {
      incidentId: incident.id,
      evidenceCount: incident.evidence.length,
      timestamp: new Date().toISOString()
    })
  }, [])

  const getSeverityColor = (severity: string) => {
    switch (severity) {
      case 'critical': return 'bg-red-100 text-red-800'
      case 'high': return 'bg-orange-100 text-orange-800'
      case 'medium': return 'bg-yellow-100 text-yellow-800'
      case 'low': return 'bg-blue-100 text-blue-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'open': return 'bg-red-100 text-red-800'
      case 'investigating': return 'bg-orange-100 text-orange-800'
      case 'resolved': return 'bg-green-100 text-green-800'
      case 'closed': return 'bg-gray-100 text-gray-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const allIncidents = [...mockIncidents, ...incidents]
    .filter(inc => !vehicleId || inc.vehicleId === vehicleId)
    .sort((a, b) => b.createdAt.getTime() - a.createdAt.getTime())

  return (
    <div className={`space-y-4 ${className}`}>
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex items-center space-x-2">
          <AlertTriangle className="w-5 h-5 text-orange-600" />
          <h3 className="text-lg font-medium text-gray-900">
            {vehicleId ? `Incidents - ${vehicleName}` : 'Incident Management'}
          </h3>
        </div>
        <Button onClick={() => setShowCreateDialog(true)}>
          <Plus className="w-4 h-4 mr-2" />
          Create Incident
        </Button>
      </div>

      {/* Incident List */}
      <div className="space-y-3">
        {allIncidents.map(incident => (
          <Card key={incident.id} className="p-4 hover:shadow-md transition-shadow">
            <div className="flex items-start justify-between">
              <div className="flex-1">
                <div className="flex items-center space-x-3 mb-2">
                  <h4 className="font-medium text-gray-900">{incident.title}</h4>
                  <Badge className={getSeverityColor(incident.severity)}>
                    {incident.severity}
                  </Badge>
                  <Badge className={getStatusColor(incident.status)}>
                    {incident.status}
                  </Badge>
                </div>
                
                <p className="text-sm text-gray-600 mb-3">{incident.description}</p>
                
                <div className="flex items-center space-x-4 text-sm text-gray-500">
                  {incident.vehicleName && (
                    <div className="flex items-center space-x-1">
                      <MapPin className="w-4 h-4" />
                      <span>{incident.vehicleName}</span>
                    </div>
                  )}
                  
                  <div className="flex items-center space-x-1">
                    <Clock className="w-4 h-4" />
                    <span>{incident.createdAt.toLocaleString()}</span>
                  </div>
                  
                  <div className="flex items-center space-x-1">
                    <User className="w-4 h-4" />
                    <span>{incident.reportedBy}</span>
                  </div>
                  
                  {incident.evidence.length > 0 && (
                    <div className="flex items-center space-x-1">
                      <FileText className="w-4 h-4" />
                      <span>{incident.evidence.length} evidence</span>
                    </div>
                  )}
                </div>
                
                {incident.tags.length > 0 && (
                  <div className="flex items-center space-x-2 mt-2">
                    <Tag className="w-4 h-4 text-gray-400" />
                    <div className="flex flex-wrap gap-1">
                      {incident.tags.map(tag => (
                        <Badge key={tag} variant="outline" size="sm">
                          {tag}
                        </Badge>
                      ))}
                    </div>
                  </div>
                )}
              </div>
              
              <div className="flex items-center space-x-2">
                <Button 
                  variant="outline" 
                  size="sm"
                  onClick={() => handleViewIncident(incident)}
                >
                  <Eye className="w-4 h-4" />
                </Button>
                
                <Button 
                  variant="outline" 
                  size="sm"
                  onClick={() => handleExportEvidence(incident)}
                  disabled={incident.evidence.length === 0}
                >
                  <Download className="w-4 h-4" />
                </Button>
              </div>
            </div>
          </Card>
        ))}
        
        {allIncidents.length === 0 && (
          <div className="text-center py-12">
            <Shield className="w-16 h-16 text-gray-400 mx-auto mb-4" />
            <h3 className="text-lg font-medium text-gray-900 mb-2">No Incidents</h3>
            <p className="text-gray-600">
              {vehicleId ? 'No incidents reported for this vehicle' : 'No incidents in the system'}
            </p>
          </div>
        )}
      </div>

      {/* Create Incident Dialog */}
      <Dialog open={showCreateDialog} onOpenChange={setShowCreateDialog}>
        <DialogContent className="max-w-2xl">
          <DialogHeader>
            <DialogTitle>Create New Incident</DialogTitle>
          </DialogHeader>
          
          <div className="space-y-4">
            {vehicleId && (
              <Alert>
                <AlertTriangle className="w-4 h-4" />
                <AlertDescription>
                  Creating incident for vehicle: <strong>{vehicleName}</strong>
                </AlertDescription>
              </Alert>
            )}
            
            <div>
              <label className="block text-sm font-medium text-gray-700 mb-2">
                Title *
              </label>
              <Input
                value={newIncident.title}
                onChange={(e) => setNewIncident(prev => ({ ...prev, title: e.target.value }))}
                placeholder="Brief description of the incident"
                required
              />
            </div>
            
            <div>
              <label className="block text-sm font-medium text-gray-700 mb-2">
                Description *
              </label>
              <Textarea
                value={newIncident.description}
                onChange={(e) => setNewIncident(prev => ({ ...prev, description: e.target.value }))}
                placeholder="Detailed description of what happened"
                rows={4}
                required
              />
            </div>
            
            <div className="grid grid-cols-2 gap-4">
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-2">
                  Severity
                </label>
                <Select
                  value={newIncident.severity}
                  onValueChange={(value) => setNewIncident(prev => ({ 
                    ...prev, 
                    severity: value as 'low' | 'medium' | 'high' | 'critical'
                  }))}
                >
                  <option value="low">Low</option>
                  <option value="medium">Medium</option>
                  <option value="high">High</option>
                  <option value="critical">Critical</option>
                </Select>
              </div>
              
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-2">
                  Type
                </label>
                <Select
                  value={newIncident.type}
                  onValueChange={(value) => setNewIncident(prev => ({ 
                    ...prev, 
                    type: value as 'safety' | 'operational' | 'technical' | 'compliance'
                  }))}
                >
                  <option value="safety">Safety</option>
                  <option value="operational">Operational</option>
                  <option value="technical">Technical</option>
                  <option value="compliance">Compliance</option>
                </Select>
              </div>
            </div>
            
            <div>
              <label className="block text-sm font-medium text-gray-700 mb-2">
                Tags (comma-separated)
              </label>
              <Input
                value={newIncident.tags}
                onChange={(e) => setNewIncident(prev => ({ ...prev, tags: e.target.value }))}
                placeholder="e.g., emergency-stop, obstacle-detection, safety-critical"
              />
            </div>
            
            <div className="flex justify-end space-x-3 pt-4">
              <Button 
                variant="outline" 
                onClick={() => setShowCreateDialog(false)}
                disabled={isLoading}
              >
                Cancel
              </Button>
              <Button 
                onClick={handleCreateIncident}
                disabled={!newIncident.title || !newIncident.description || isLoading}
              >
                {isLoading ? 'Creating...' : 'Create Incident'}
              </Button>
            </div>
          </div>
        </DialogContent>
      </Dialog>

      {/* View Incident Dialog */}
      <Dialog open={showViewDialog} onOpenChange={setShowViewDialog}>
        <DialogContent className="max-w-4xl max-h-[90vh] overflow-y-auto">
          <DialogHeader>
            <DialogTitle>Incident Details</DialogTitle>
          </DialogHeader>
          
          {selectedIncident && (
            <div className="space-y-6">
              {/* Basic Info */}
              <div>
                <div className="flex items-center space-x-3 mb-4">
                  <h3 className="text-lg font-medium text-gray-900">{selectedIncident.title}</h3>
                  <Badge className={getSeverityColor(selectedIncident.severity)}>
                    {selectedIncident.severity}
                  </Badge>
                  <Badge className={getStatusColor(selectedIncident.status)}>
                    {selectedIncident.status}
                  </Badge>
                </div>
                
                <p className="text-gray-600 mb-4">{selectedIncident.description}</p>
                
                <div className="grid grid-cols-2 gap-4 text-sm">
                  <div>
                    <span className="font-medium">Vehicle:</span> {selectedIncident.vehicleName || 'N/A'}
                  </div>
                  <div>
                    <span className="font-medium">Type:</span> {selectedIncident.type}
                  </div>
                  <div>
                    <span className="font-medium">Reported By:</span> {selectedIncident.reportedBy}
                  </div>
                  <div>
                    <span className="font-medium">Assigned To:</span> {selectedIncident.assignedTo || 'Unassigned'}
                  </div>
                  <div>
                    <span className="font-medium">Created:</span> {selectedIncident.createdAt.toLocaleString()}
                  </div>
                  <div>
                    <span className="font-medium">Updated:</span> {selectedIncident.updatedAt.toLocaleString()}
                  </div>
                </div>
              </div>
              
              {/* Evidence */}
              {selectedIncident.evidence.length > 0 && (
                <div>
                  <h4 className="text-lg font-medium text-gray-900 mb-3">Evidence</h4>
                  <div className="grid grid-cols-1 gap-3">
                    {selectedIncident.evidence.map(evidence => (
                      <Card key={evidence.id} className="p-3">
                        <div className="flex items-center justify-between">
                          <div className="flex items-center space-x-3">
                            <div className="p-2 bg-blue-100 rounded">
                              {evidence.type === 'photo' && <Camera className="w-4 h-4 text-blue-600" />}
                              {evidence.type === 'document' && <FileText className="w-4 h-4 text-blue-600" />}
                              {evidence.type === 'log' && <Archive className="w-4 h-4 text-blue-600" />}
                            </div>
                            <div>
                              <div className="font-medium text-gray-900">{evidence.filename}</div>
                              <div className="text-sm text-gray-600">{evidence.description}</div>
                              <div className="text-xs text-gray-500">
                                {evidence.capturedAt.toLocaleString()} • {(evidence.size / 1024 / 1024).toFixed(2)} MB
                              </div>
                            </div>
                          </div>
                          <div className="flex items-center space-x-2">
                            <Button variant="outline" size="sm">
                              <Eye className="w-4 h-4" />
                            </Button>
                            <Button variant="outline" size="sm">
                              <Download className="w-4 h-4" />
                            </Button>
                          </div>
                        </div>
                      </Card>
                    ))}
                  </div>
                </div>
              )}
              
              {/* Timeline */}
              <div>
                <h4 className="text-lg font-medium text-gray-900 mb-3">Timeline</h4>
                <div className="space-y-3">
                  {selectedIncident.timeline.map(event => (
                    <div key={event.id} className="flex items-start space-x-3">
                      <div className="w-2 h-2 bg-blue-500 rounded-full mt-2" />
                      <div className="flex-1">
                        <div className="font-medium text-gray-900">{event.description}</div>
                        <div className="text-sm text-gray-600">
                          {event.timestamp.toLocaleString()} • {event.user}
                        </div>
                      </div>
                    </div>
                  ))}
                </div>
              </div>
              
              <div className="flex justify-end space-x-3 pt-4 border-t">
                <Button 
                  variant="outline"
                  onClick={() => handleExportEvidence(selectedIncident)}
                  disabled={selectedIncident.evidence.length === 0}
                >
                  <Download className="w-4 h-4 mr-2" />
                  Export Evidence
                </Button>
                <Button variant="outline">
                  <Edit className="w-4 h-4 mr-2" />
                  Edit Incident
                </Button>
              </div>
            </div>
          )}
        </DialogContent>
      </Dialog>
    </div>
  )
}

export default IncidentManager
