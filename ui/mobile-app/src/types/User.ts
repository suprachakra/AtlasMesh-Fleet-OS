export enum UserRole {
  ADMIN = 'admin',
  FLEET_MANAGER = 'fleet_manager',
  DRIVER = 'driver',
  TECHNICIAN = 'technician',
  DISPATCHER = 'dispatcher',
  SUPERVISOR = 'supervisor'
}

export interface User {
  id: string;
  username: string;
  email: string;
  firstName: string;
  lastName: string;
  role: UserRole;
  permissions: string[];
  avatar?: string;
  phone?: string;
  employeeId?: string;
  department?: string;
  location?: {
    latitude: number;
    longitude: number;
    address?: string;
  };
  preferences: {
    language: string;
    theme: 'light' | 'dark' | 'auto';
    notifications: {
      push: boolean;
      email: boolean;
      sms: boolean;
    };
    units: {
      distance: 'km' | 'miles';
      temperature: 'celsius' | 'fahrenheit';
      fuel: 'liters' | 'gallons';
    };
  };
  status: 'active' | 'inactive' | 'suspended';
  lastLogin?: Date;
  createdAt: Date;
  updatedAt: Date;
}

export interface AuthResponse {
  user: User;
  token: string;
  refreshToken: string;
  expiresAt: Date;
}

export interface LoginCredentials {
  username: string;
  password: string;
  rememberMe?: boolean;
}
