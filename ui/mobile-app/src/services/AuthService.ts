import AsyncStorage from '@react-native-async-storage/async-storage';
import { User, AuthResponse, LoginCredentials } from '../types/User';

const API_BASE_URL = 'https://api.atlasmesh.ae';
const AUTH_TOKEN_KEY = 'auth_token';
const REFRESH_TOKEN_KEY = 'refresh_token';
const USER_DATA_KEY = 'user_data';

export class AuthService {
  private static currentUser: User | null = null;
  private static authToken: string | null = null;

  static async login(credentials: LoginCredentials): Promise<AuthResponse> {
    try {
      const response = await fetch(`${API_BASE_URL}/api/v1/auth/login`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(credentials),
      });

      if (!response.ok) {
        throw new Error('Login failed');
      }

      const authResponse: AuthResponse = await response.json();
      
      // Store authentication data
      await this.storeAuthData(authResponse);
      
      this.currentUser = authResponse.user;
      this.authToken = authResponse.token;

      return authResponse;
    } catch (error) {
      console.error('Login error:', error);
      throw error;
    }
  }

  static async logout(): Promise<void> {
    try {
      // Call logout endpoint if token exists
      if (this.authToken) {
        await fetch(`${API_BASE_URL}/api/v1/auth/logout`, {
          method: 'POST',
          headers: {
            'Authorization': `Bearer ${this.authToken}`,
            'Content-Type': 'application/json',
          },
        });
      }
    } catch (error) {
      console.error('Logout error:', error);
    } finally {
      // Clear local data regardless of API call success
      await this.clearAuthData();
      this.currentUser = null;
      this.authToken = null;
    }
  }

  static async getCurrentUser(): Promise<User | null> {
    if (this.currentUser) {
      return this.currentUser;
    }

    try {
      const userData = await AsyncStorage.getItem(USER_DATA_KEY);
      const token = await AsyncStorage.getItem(AUTH_TOKEN_KEY);

      if (userData && token) {
        this.currentUser = JSON.parse(userData);
        this.authToken = token;
        
        // Verify token is still valid
        const isValid = await this.verifyToken();
        if (!isValid) {
          await this.clearAuthData();
          return null;
        }

        return this.currentUser;
      }
    } catch (error) {
      console.error('Get current user error:', error);
    }

    return null;
  }

  static async getAuthToken(): Promise<string | null> {
    if (this.authToken) {
      return this.authToken;
    }

    try {
      const token = await AsyncStorage.getItem(AUTH_TOKEN_KEY);
      if (token) {
        this.authToken = token;
        return token;
      }
    } catch (error) {
      console.error('Get auth token error:', error);
    }

    return null;
  }

  static async refreshToken(): Promise<string | null> {
    try {
      const refreshToken = await AsyncStorage.getItem(REFRESH_TOKEN_KEY);
      if (!refreshToken) {
        throw new Error('No refresh token available');
      }

      const response = await fetch(`${API_BASE_URL}/api/v1/auth/refresh`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ refreshToken }),
      });

      if (!response.ok) {
        throw new Error('Token refresh failed');
      }

      const authResponse: AuthResponse = await response.json();
      
      // Update stored tokens
      await AsyncStorage.setItem(AUTH_TOKEN_KEY, authResponse.token);
      await AsyncStorage.setItem(REFRESH_TOKEN_KEY, authResponse.refreshToken);
      
      this.authToken = authResponse.token;

      return authResponse.token;
    } catch (error) {
      console.error('Token refresh error:', error);
      // If refresh fails, clear auth data
      await this.clearAuthData();
      return null;
    }
  }

  static async verifyToken(): Promise<boolean> {
    try {
      const token = await this.getAuthToken();
      if (!token) {
        return false;
      }

      const response = await fetch(`${API_BASE_URL}/api/v1/auth/verify`, {
        method: 'GET',
        headers: {
          'Authorization': `Bearer ${token}`,
        },
      });

      return response.ok;
    } catch (error) {
      console.error('Token verification error:', error);
      return false;
    }
  }

  static async updateUserProfile(updates: Partial<User>): Promise<User> {
    try {
      const token = await this.getAuthToken();
      if (!token) {
        throw new Error('No authentication token');
      }

      const response = await fetch(`${API_BASE_URL}/api/v1/auth/profile`, {
        method: 'PUT',
        headers: {
          'Authorization': `Bearer ${token}`,
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(updates),
      });

      if (!response.ok) {
        throw new Error('Profile update failed');
      }

      const updatedUser: User = await response.json();
      
      // Update local user data
      this.currentUser = updatedUser;
      await AsyncStorage.setItem(USER_DATA_KEY, JSON.stringify(updatedUser));

      return updatedUser;
    } catch (error) {
      console.error('Profile update error:', error);
      throw error;
    }
  }

  static async changePassword(currentPassword: string, newPassword: string): Promise<void> {
    try {
      const token = await this.getAuthToken();
      if (!token) {
        throw new Error('No authentication token');
      }

      const response = await fetch(`${API_BASE_URL}/api/v1/auth/change-password`, {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${token}`,
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          currentPassword,
          newPassword,
        }),
      });

      if (!response.ok) {
        throw new Error('Password change failed');
      }
    } catch (error) {
      console.error('Password change error:', error);
      throw error;
    }
  }

  private static async storeAuthData(authResponse: AuthResponse): Promise<void> {
    try {
      await AsyncStorage.multiSet([
        [AUTH_TOKEN_KEY, authResponse.token],
        [REFRESH_TOKEN_KEY, authResponse.refreshToken],
        [USER_DATA_KEY, JSON.stringify(authResponse.user)],
      ]);
    } catch (error) {
      console.error('Store auth data error:', error);
      throw error;
    }
  }

  private static async clearAuthData(): Promise<void> {
    try {
      await AsyncStorage.multiRemove([
        AUTH_TOKEN_KEY,
        REFRESH_TOKEN_KEY,
        USER_DATA_KEY,
      ]);
    } catch (error) {
      console.error('Clear auth data error:', error);
    }
  }

  // Helper method for making authenticated API calls
  static async makeAuthenticatedRequest(
    url: string,
    options: RequestInit = {}
  ): Promise<Response> {
    let token = await this.getAuthToken();
    
    if (!token) {
      throw new Error('No authentication token');
    }

    const response = await fetch(url, {
      ...options,
      headers: {
        ...options.headers,
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json',
      },
    });

    // If token is expired, try to refresh
    if (response.status === 401) {
      token = await this.refreshToken();
      if (token) {
        // Retry the request with new token
        return fetch(url, {
          ...options,
          headers: {
            ...options.headers,
            'Authorization': `Bearer ${token}`,
            'Content-Type': 'application/json',
          },
        });
      } else {
        throw new Error('Authentication failed');
      }
    }

    return response;
  }
}
