import React, { useState } from 'react'
import { X, Upload, Plus, Trash2 } from 'lucide-react'
import { Button } from '../ui/Button'
import { Input } from '../ui/Input'
import { Select } from '../ui/Select'
import { Dialog, DialogContent, DialogHeader, DialogTitle } from '../ui/Dialog'
import { Badge } from '../ui/Badge'

interface WorklogModalProps {
  open: boolean
  onClose: () => void
  vehicleId: string
  onSubmit: (worklog: WorklogFormData) => void
}

interface WorklogFormData {
  type: string
  title: string
  description: string
  startTime: string
  endTime: string
  technician: string
  owner: string
  approver: string
  jiraTicket: string
  ecoNumber: string
  priority: string
  status: string
  parts: Array<{
    partNumber: string
    description: string
    quantity: number
    cost: number
  }>
}

export const WorklogModal: React.FC<WorklogModalProps> = ({ open, onClose, vehicleId, onSubmit }) => {
  const [formData, setFormData] = useState<WorklogFormData>({
    type: 'maintenance',
    title: '',
    description: '',
    startTime: new Date().toISOString().slice(0, 16),
    endTime: '',
    technician: '',
    owner: '',
    approver: '',
    jiraTicket: '',
    ecoNumber: '',
    priority: 'medium',
    status: 'scheduled',
    parts: []
  })

  const handleSubmit = () => {
    onSubmit(formData)
    onClose()
  }

  const addPart = () => {
    setFormData({
      ...formData,
      parts: [...formData.parts, { partNumber: '', description: '', quantity: 1, cost: 0 }]
    })
  }

  const removePart = (index: number) => {
    setFormData({
      ...formData,
      parts: formData.parts.filter((_, i) => i !== index)
    })
  }

  const updatePart = (index: number, field: string, value: any) => {
    const updatedParts = formData.parts.map((part, i) => 
      i === index ? { ...part, [field]: value } : part
    )
    setFormData({ ...formData, parts: updatedParts })
  }

  return (
    <Dialog open={open} onOpenChange={onClose}>
      <DialogContent className="max-w-4xl max-h-[90vh] overflow-y-auto">
        <DialogHeader>
          <DialogTitle>Add Work Log</DialogTitle>
        </DialogHeader>

        <div className="space-y-6">
          {/* Type of Work */}
          <div className="grid grid-cols-2 gap-4">
            <div>
              <label className="block text-sm font-medium text-gray-700 mb-1">
                Type of Work <span className="text-red-500">*</span>
              </label>
              <Select
                value={formData.type}
                onValueChange={(value) => setFormData({ ...formData, type: value })}
              >
                <option value="maintenance">Maintenance</option>
                <option value="repair">Repair</option>
                <option value="inspection">Inspection</option>
                <option value="upgrade">Upgrade</option>
                <option value="incident">Incident</option>
                <option value="testing">Testing</option>
                <option value="calibration">Calibration</option>
                <option value="cleaning">Cleaning</option>
                <option value="documentation">Documentation</option>
                <option value="other">Other</option>
              </Select>
            </div>

            <div>
              <label className="block text-sm font-medium text-gray-700 mb-1">
                Priority
              </label>
              <Select
                value={formData.priority}
                onValueChange={(value) => setFormData({ ...formData, priority: value })}
              >
                <option value="low">Low</option>
                <option value="medium">Medium</option>
                <option value="high">High</option>
                <option value="critical">Critical</option>
              </Select>
            </div>
          </div>

          {/* Title */}
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Title <span className="text-red-500">*</span>
            </label>
            <Input
              value={formData.title}
              onChange={(e) => setFormData({ ...formData, title: e.target.value })}
              placeholder="Brief description of the work"
            />
          </div>

          {/* Work Description (Rich Text Editor) */}
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Work Description <span className="text-red-500">*</span>
            </label>
            <div className="border border-gray-300 rounded-md">
              {/* Rich Text Toolbar */}
              <div className="flex items-center space-x-1 p-2 border-b border-gray-200 bg-gray-50">
                <Button variant="ghost" size="sm" title="Bold">
                  <strong>B</strong>
                </Button>
                <Button variant="ghost" size="sm" title="Italic">
                  <em>I</em>
                </Button>
                <Button variant="ghost" size="sm" title="Underline">
                  <u>U</u>
                </Button>
                <div className="w-px h-6 bg-gray-300 mx-2" />
                <Button variant="ghost" size="sm" title="Bullet List">
                  •
                </Button>
                <Button variant="ghost" size="sm" title="Numbered List">
                  1.
                </Button>
                <div className="w-px h-6 bg-gray-300 mx-2" />
                <Button variant="ghost" size="sm" title="Link">
                  🔗
                </Button>
                <Button variant="ghost" size="sm" title="Image">
                  🖼️
                </Button>
              </div>
              {/* Rich Text Area */}
              <textarea
                value={formData.description}
                onChange={(e) => setFormData({ ...formData, description: e.target.value })}
                className="w-full p-3 min-h-[150px] focus:outline-none resize-none"
                placeholder="Detailed description of work performed..."
              />
            </div>
          </div>

          {/* Timing */}
          <div className="grid grid-cols-2 gap-4">
            <div>
              <label className="block text-sm font-medium text-gray-700 mb-1">
                Start Time <span className="text-red-500">*</span>
              </label>
              <Input
                type="datetime-local"
                value={formData.startTime}
                onChange={(e) => setFormData({ ...formData, startTime: e.target.value })}
              />
            </div>

            <div>
              <label className="block text-sm font-medium text-gray-700 mb-1">
                ETA End Time
              </label>
              <Input
                type="datetime-local"
                value={formData.endTime}
                onChange={(e) => setFormData({ ...formData, endTime: e.target.value })}
              />
            </div>
          </div>

          {/* Integration Fields */}
          <div className="grid grid-cols-2 gap-4">
            <div>
              <label className="block text-sm font-medium text-gray-700 mb-1">
                JIRA Ticket
              </label>
              <Input
                value={formData.jiraTicket}
                onChange={(e) => setFormData({ ...formData, jiraTicket: e.target.value })}
                placeholder="FLEET-1234"
              />
            </div>

            <div>
              <label className="block text-sm font-medium text-gray-700 mb-1">
                ECO#
              </label>
              <Input
                value={formData.ecoNumber}
                onChange={(e) => setFormData({ ...formData, ecoNumber: e.target.value })}
                placeholder="ECO-2025-001"
              />
            </div>
          </div>

          {/* Personnel */}
          <div className="grid grid-cols-3 gap-4">
            <div>
              <label className="block text-sm font-medium text-gray-700 mb-1">
                Technician <span className="text-red-500">*</span>
              </label>
              <Input
                value={formData.technician}
                onChange={(e) => setFormData({ ...formData, technician: e.target.value })}
                placeholder="John Doe"
              />
            </div>

            <div>
              <label className="block text-sm font-medium text-gray-700 mb-1">
                Owner <span className="text-red-500">*</span>
              </label>
              <Input
                type="email"
                value={formData.owner}
                onChange={(e) => setFormData({ ...formData, owner: e.target.value })}
                placeholder="owner@atlasmesh.ae"
              />
            </div>

            <div>
              <label className="block text-sm font-medium text-gray-700 mb-1">
                Approver
              </label>
              <Input
                type="email"
                value={formData.approver}
                onChange={(e) => setFormData({ ...formData, approver: e.target.value })}
                placeholder="approver@atlasmesh.ae"
              />
            </div>
          </div>

          {/* Parts Section */}
          <div>
            <div className="flex items-center justify-between mb-2">
              <label className="block text-sm font-medium text-gray-700">Parts Used</label>
              <Button variant="outline" size="sm" onClick={addPart}>
                <Plus className="w-4 h-4 mr-1" />
                Add Part
              </Button>
            </div>

            {formData.parts.length > 0 && (
              <div className="space-y-2">
                {formData.parts.map((part, index) => (
                  <div key={index} className="grid grid-cols-5 gap-2 p-2 border border-gray-200 rounded">
                    <Input
                      placeholder="Part #"
                      value={part.partNumber}
                      onChange={(e) => updatePart(index, 'partNumber', e.target.value)}
                    />
                    <Input
                      placeholder="Description"
                      value={part.description}
                      onChange={(e) => updatePart(index, 'description', e.target.value)}
                      className="col-span-2"
                    />
                    <Input
                      type="number"
                      placeholder="Qty"
                      value={part.quantity}
                      onChange={(e) => updatePart(index, 'quantity', parseInt(e.target.value))}
                    />
                    <div className="flex items-center space-x-2">
                      <Input
                        type="number"
                        placeholder="Cost"
                        value={part.cost}
                        onChange={(e) => updatePart(index, 'cost', parseFloat(e.target.value))}
                      />
                      <Button variant="ghost" size="sm" onClick={() => removePart(index)}>
                        <Trash2 className="w-4 h-4 text-red-500" />
                      </Button>
                    </div>
                  </div>
                ))}
              </div>
            )}
          </div>

          {/* Action Buttons */}
          <div className="flex justify-end space-x-3 pt-4 border-t border-gray-200">
            <Button variant="outline" onClick={onClose}>
              CANCEL
            </Button>
            <Button 
              onClick={handleSubmit}
              disabled={!formData.title || !formData.description || !formData.owner}
            >
              SUBMIT
            </Button>
          </div>
        </div>
      </DialogContent>
    </Dialog>
  )
}

export default WorklogModal

