// AtlasMesh Fleet OS - Shared Component Library
// Centralized exports for reusable UI components across the application

// Status Components
export {
  StatusIndicator,
  VehicleStatusIndicator,
  SystemStatusIndicator,
  EmergencyStatusIndicator,
  StatusIndicatorWithCount,
  type StatusType,
  type StatusSize,
  type StatusIndicatorProps
} from './StatusIndicator'

// Data Display Components
export {
  DataTable,
  type Column,
  type DataTableProps,
  type SortDirection
} from './DataTable'

// Form Components
export {
  Input,
  Select,
  Textarea,
  Checkbox,
  RadioGroup,
  Button,
  FormGroup,
  UAEPhoneInput,
  EmiratesIDInput,
  BilingualInput,
  type InputProps,
  type SelectProps,
  type TextareaProps,
  type CheckboxProps,
  type RadioOption,
  type RadioGroupProps,
  type ButtonProps,
  type FormGroupProps,
  type BilingualInputProps
} from './FormComponents'

// Re-export default components for convenience
export { default as StatusIndicatorComponent } from './StatusIndicator'
export { default as DataTableComponent } from './DataTable'
export { default as FormComponents } from './FormComponents'
