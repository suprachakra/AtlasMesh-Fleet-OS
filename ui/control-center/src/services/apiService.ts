/**
 * API Service for AtlasMesh Fleet OS Control Center
 * 
 * Provides centralized API communication with backend services
 * Handles authentication, error handling, and data transformation
 */

export interface ApiResponse<T = any> {
  data: T;
  status: number;
  message?: string;
  error?: string;
}

export interface ApiError {
  message: string;
  status: number;
  code?: string;
}

export class ApiService {
  private baseURL: string;
  private authToken: string | null;

  constructor(baseURL: string = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8080') {
    this.baseURL = baseURL;
    this.authToken = localStorage.getItem('authToken');
  }

  /**
   * Set authentication token
   */
  setAuthToken(token: string): void {
    this.authToken = token;
    localStorage.setItem('authToken', token);
  }

  /**
   * Clear authentication token
   */
  clearAuthToken(): void {
    this.authToken = null;
    localStorage.removeItem('authToken');
  }

  /**
   * Get headers for API requests
   */
  private getHeaders(): HeadersInit {
    const headers: HeadersInit = {
      'Content-Type': 'application/json',
    };

    if (this.authToken) {
      headers['Authorization'] = `Bearer ${this.authToken}`;
    }

    return headers;
  }

  /**
   * Make HTTP request with error handling
   */
  private async request<T>(
    endpoint: string,
    options: RequestInit = {}
  ): Promise<ApiResponse<T>> {
    const url = `${this.baseURL}${endpoint}`;
    
    try {
      const response = await fetch(url, {
        ...options,
        headers: {
          ...this.getHeaders(),
          ...options.headers,
        },
      });

      const data = await response.json();

      if (!response.ok) {
        throw new ApiError(
          data.message || data.error || 'Request failed',
          response.status,
          data.code
        );
      }

      return {
        data,
        status: response.status,
        message: data.message,
      };
    } catch (error) {
      if (error instanceof ApiError) {
        throw error;
      }
      
      throw new ApiError(
        error instanceof Error ? error.message : 'Network error',
        0,
        'NETWORK_ERROR'
      );
    }
  }

  // Fleet Manager API

  /**
   * Get all vehicles in fleet
   */
  async getVehicles(fleetId?: string): Promise<ApiResponse<any[]>> {
    const endpoint = fleetId ? `/api/v1/fleet/vehicles?fleet_id=${fleetId}` : '/api/v1/fleet/vehicles';
    return this.request<any[]>(endpoint);
  }

  /**
   * Get specific vehicle details
   */
  async getVehicle(vehicleId: string): Promise<ApiResponse<any>> {
    return this.request<any>(`/api/v1/fleet/vehicles/${vehicleId}`);
  }

  /**
   * Create new vehicle
   */
  async createVehicle(vehicleData: any): Promise<ApiResponse<any>> {
    return this.request<any>('/api/v1/fleet/vehicles', {
      method: 'POST',
      body: JSON.stringify(vehicleData),
    });
  }

  /**
   * Update vehicle
   */
  async updateVehicle(vehicleId: string, vehicleData: any): Promise<ApiResponse<any>> {
    return this.request<any>(`/api/v1/fleet/vehicles/${vehicleId}`, {
      method: 'PUT',
      body: JSON.stringify(vehicleData),
    });
  }

  /**
   * Delete vehicle
   */
  async deleteVehicle(vehicleId: string): Promise<ApiResponse<void>> {
    return this.request<void>(`/api/v1/fleet/vehicles/${vehicleId}`, {
      method: 'DELETE',
    });
  }

  /**
   * Get vehicle status
   */
  async getVehicleStatus(vehicleId: string): Promise<ApiResponse<any>> {
    return this.request<any>(`/api/v1/fleet/vehicles/${vehicleId}/status`);
  }

  /**
   * Update vehicle location
   */
  async updateVehicleLocation(vehicleId: string, location: any): Promise<ApiResponse<any>> {
    return this.request<any>(`/api/v1/fleet/vehicles/${vehicleId}/location`, {
      method: 'PUT',
      body: JSON.stringify(location),
    });
  }

  /**
   * Dispatch vehicle
   */
  async dispatchVehicle(vehicleId: string, dispatchData: any): Promise<ApiResponse<any>> {
    return this.request<any>(`/api/v1/fleet/vehicles/${vehicleId}/dispatch`, {
      method: 'POST',
      body: JSON.stringify(dispatchData),
    });
  }

  /**
   * Recall vehicle
   */
  async recallVehicle(vehicleId: string, reason: string): Promise<ApiResponse<any>> {
    return this.request<any>(`/api/v1/fleet/vehicles/${vehicleId}/recall`, {
      method: 'POST',
      body: JSON.stringify({ reason }),
    });
  }

  /**
   * Get fleet utilization metrics
   */
  async getFleetUtilization(fleetId: string, timeframe?: string): Promise<ApiResponse<any>> {
    const endpoint = `/api/v1/fleet/fleets/${fleetId}/utilization${timeframe ? `?timeframe=${timeframe}` : ''}`;
    return this.request<any>(endpoint);
  }

  /**
   * Get vehicle health metrics
   */
  async getVehicleHealthMetrics(vehicleId: string): Promise<ApiResponse<any>> {
    return this.request<any>(`/api/v1/fleet/vehicles/${vehicleId}/health`);
  }

  /**
   * Get operational metrics
   */
  async getOperationalMetrics(fleetId?: string): Promise<ApiResponse<any>> {
    const endpoint = fleetId ? `/api/v1/fleet/operational-metrics?fleet_id=${fleetId}` : '/api/v1/fleet/operational-metrics';
    return this.request<any>(endpoint);
  }

  // Analytics API

  /**
   * Get fleet KPIs
   */
  async getFleetKPIs(timeframe?: string): Promise<ApiResponse<any>> {
    const endpoint = `/api/v1/analytics/kpis${timeframe ? `?timeframe=${timeframe}` : ''}`;
    return this.request<any>(endpoint);
  }

  /**
   * Get operational metrics
   */
  async getAnalyticsOperationalMetrics(): Promise<ApiResponse<any>> {
    return this.request<any>('/api/v1/analytics/operational');
  }

  /**
   * Get financial metrics
   */
  async getFinancialMetrics(): Promise<ApiResponse<any>> {
    return this.request<any>('/api/v1/analytics/financial');
  }

  /**
   * Get safety metrics
   */
  async getSafetyMetrics(): Promise<ApiResponse<any>> {
    return this.request<any>('/api/v1/analytics/safety');
  }

  /**
   * Execute custom analytics query
   */
  async executeAnalyticsQuery(query: string, parameters?: any): Promise<ApiResponse<any>> {
    return this.request<any>('/api/v1/analytics/query', {
      method: 'POST',
      body: JSON.stringify({ query, parameters }),
    });
  }

  /**
   * Get real-time metrics
   */
  async getRealTimeMetrics(): Promise<ApiResponse<any>> {
    return this.request<any>('/api/v1/analytics/realtime');
  }

  // External Integrations API

  /**
   * Get UAE government vehicle registration
   */
  async getVehicleRegistration(vehicleId: string): Promise<ApiResponse<any>> {
    return this.request<any>(`/api/v1/uae/vehicle-registration/${vehicleId}`);
  }

  /**
   * Check compliance
   */
  async checkCompliance(vehicleId: string): Promise<ApiResponse<any>> {
    return this.request<any>(`/api/v1/uae/compliance-check/${vehicleId}`);
  }

  /**
   * Generate compliance report
   */
  async generateComplianceReport(vehicleIds: string[], reportType: string, format: string): Promise<ApiResponse<any>> {
    return this.request<any>('/api/v1/uae/generate-report', {
      method: 'POST',
      body: JSON.stringify({ vehicle_ids: vehicleIds, report_type: reportType, format }),
    });
  }

  /**
   * Get current weather
   */
  async getCurrentWeather(location: string): Promise<ApiResponse<any>> {
    return this.request<any>(`/api/v1/weather/current/${encodeURIComponent(location)}`);
  }

  /**
   * Get weather forecast
   */
  async getWeatherForecast(location: string): Promise<ApiResponse<any>> {
    return this.request<any>(`/api/v1/weather/forecast/${encodeURIComponent(location)}`);
  }

  /**
   * Get weather alerts
   */
  async getWeatherAlerts(location: string): Promise<ApiResponse<any>> {
    return this.request<any>(`/api/v1/weather/alerts/${encodeURIComponent(location)}`);
  }

  /**
   * Geocode address
   */
  async geocodeAddress(address: string): Promise<ApiResponse<any>> {
    return this.request<any>(`/api/v1/maps/geocode/${encodeURIComponent(address)}`);
  }

  /**
   * Get route between two points
   */
  async getRoute(origin: string, destination: string, mode: string = 'driving'): Promise<ApiResponse<any>> {
    return this.request<any>('/api/v1/maps/route', {
      method: 'POST',
      body: JSON.stringify({ origin, destination, mode }),
    });
  }

  /**
   * Get traffic information
   */
  async getTrafficInfo(location: string): Promise<ApiResponse<any>> {
    return this.request<any>(`/api/v1/maps/traffic/${encodeURIComponent(location)}`);
  }

  /**
   * Get ERP orders
   */
  async getERPOrders(): Promise<ApiResponse<any[]>> {
    return this.request<any[]>('/api/v1/erp/orders');
  }

  /**
   * Get specific ERP order
   */
  async getERPOrder(orderId: string): Promise<ApiResponse<any>> {
    return this.request<any>(`/api/v1/erp/orders/${orderId}`);
  }

  /**
   * Sync ERP data
   */
  async syncERPData(): Promise<ApiResponse<any>> {
    return this.request<any>('/api/v1/erp/sync', {
      method: 'POST',
    });
  }

  /**
   * Get WMS inventory
   */
  async getWMSInventory(warehouseId: string): Promise<ApiResponse<any>> {
    return this.request<any>(`/api/v1/wms/inventory/${warehouseId}`);
  }

  /**
   * Get WMS warehouses
   */
  async getWMSWarehouses(): Promise<ApiResponse<any[]>> {
    return this.request<any[]>('/api/v1/wms/warehouses');
  }

  /**
   * Sync WMS data
   */
  async syncWMSData(): Promise<ApiResponse<any>> {
    return this.request<any>('/api/v1/wms/sync', {
      method: 'POST',
    });
  }

  /**
   * Get sync status
   */
  async getSyncStatus(): Promise<ApiResponse<any>> {
    return this.request<any>('/api/v1/sync/status');
  }

  /**
   * Trigger sync
   */
  async triggerSync(services: string[]): Promise<ApiResponse<any>> {
    return this.request<any>('/api/v1/sync/trigger', {
      method: 'POST',
      body: JSON.stringify({ services }),
    });
  }

  /**
   * Get sync history
   */
  async getSyncHistory(): Promise<ApiResponse<any[]>> {
    return this.request<any[]>('/api/v1/sync/history');
  }

  // Authentication API

  /**
   * Login user
   */
  async login(email: string, password: string): Promise<ApiResponse<any>> {
    return this.request<any>('/api/v1/auth/login', {
      method: 'POST',
      body: JSON.stringify({ email, password }),
    });
  }

  /**
   * Logout user
   */
  async logout(): Promise<ApiResponse<void>> {
    return this.request<void>('/api/v1/auth/logout', {
      method: 'POST',
    });
  }

  /**
   * Refresh token
   */
  async refreshToken(): Promise<ApiResponse<any>> {
    return this.request<any>('/api/v1/auth/refresh', {
      method: 'POST',
    });
  }

  /**
   * Get user profile
   */
  async getUserProfile(): Promise<ApiResponse<any>> {
    return this.request<any>('/api/v1/auth/profile');
  }

  // Health checks

  /**
   * Check service health
   */
  async checkHealth(service?: string): Promise<ApiResponse<any>> {
    const endpoint = service ? `/api/v1/health/${service}` : '/api/v1/health';
    return this.request<any>(endpoint);
  }

  /**
   * Check service readiness
   */
  async checkReadiness(service?: string): Promise<ApiResponse<any>> {
    const endpoint = service ? `/api/v1/ready/${service}` : '/api/v1/ready';
    return this.request<any>(endpoint);
  }
}

// Custom error class
class ApiError extends Error {
  constructor(
    message: string,
    public status: number,
    public code?: string
  ) {
    super(message);
    this.name = 'ApiError';
  }
}

// Create singleton instance
export const apiService = new ApiService();

// Export types
export type { ApiResponse, ApiError };
