import React, { useState } from 'react';
import {
  View,
  Text,
  TextInput,
  TouchableOpacity,
  StyleSheet,
  Alert,
  KeyboardAvoidingView,
  Platform,
  ScrollView,
  Image,
} from 'react-native';
import { Ionicons } from '@expo/vector-icons';
import { AuthService } from '../services/AuthService';
import { UserRole } from '../types/User';
import { theme } from '../theme/theme';

interface LoginScreenProps {
  onLogin: (user: any) => void;
}

export default function LoginScreen({ onLogin }: LoginScreenProps) {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [showPassword, setShowPassword] = useState(false);

  const handleLogin = async () => {
    if (!email || !password) {
      Alert.alert('Error', 'Please enter both email and password');
      return;
    }

    setIsLoading(true);
    try {
      const user = await AuthService.login(email, password);
      onLogin(user);
    } catch (error) {
      Alert.alert('Login Failed', error instanceof Error ? error.message : 'Unknown error occurred');
    } finally {
      setIsLoading(false);
    }
  };

  const handleDemoLogin = async (role: UserRole) => {
    setIsLoading(true);
    try {
      const user = await AuthService.demoLogin(role);
      onLogin(user);
    } catch (error) {
      Alert.alert('Demo Login Failed', error instanceof Error ? error.message : 'Unknown error occurred');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <KeyboardAvoidingView 
      style={styles.container} 
      behavior={Platform.OS === 'ios' ? 'padding' : 'height'}
    >
      <ScrollView contentContainerStyle={styles.scrollContainer}>
        {/* Header */}
        <View style={styles.header}>
          <Image 
            source={{ uri: 'https://via.placeholder.com/120x120/1976d2/ffffff?text=AM' }}
            style={styles.logo}
          />
          <Text style={styles.title}>AtlasMesh Fleet OS</Text>
          <Text style={styles.subtitle}>Mobile Operations Center</Text>
        </View>

        {/* Login Form */}
        <View style={styles.formContainer}>
          <Text style={styles.formTitle}>Sign In</Text>
          
          <View style={styles.inputContainer}>
            <Ionicons name="mail-outline" size={20} color={theme.colors.primary} style={styles.inputIcon} />
            <TextInput
              style={styles.input}
              placeholder="Email"
              placeholderTextColor="#999"
              value={email}
              onChangeText={setEmail}
              keyboardType="email-address"
              autoCapitalize="none"
              autoCorrect={false}
            />
          </View>

          <View style={styles.inputContainer}>
            <Ionicons name="lock-closed-outline" size={20} color={theme.colors.primary} style={styles.inputIcon} />
            <TextInput
              style={styles.input}
              placeholder="Password"
              placeholderTextColor="#999"
              value={password}
              onChangeText={setPassword}
              secureTextEntry={!showPassword}
              autoCapitalize="none"
              autoCorrect={false}
            />
            <TouchableOpacity 
              style={styles.eyeIcon}
              onPress={() => setShowPassword(!showPassword)}
            >
              <Ionicons 
                name={showPassword ? "eye-outline" : "eye-off-outline"} 
                size={20} 
                color="#999" 
              />
            </TouchableOpacity>
          </View>

          <TouchableOpacity 
            style={[styles.loginButton, isLoading && styles.loginButtonDisabled]}
            onPress={handleLogin}
            disabled={isLoading}
          >
            <Text style={styles.loginButtonText}>
              {isLoading ? 'Signing In...' : 'Sign In'}
            </Text>
          </TouchableOpacity>

          <TouchableOpacity style={styles.forgotPassword}>
            <Text style={styles.forgotPasswordText}>Forgot Password?</Text>
          </TouchableOpacity>
        </View>

        {/* Demo Login Options */}
        <View style={styles.demoContainer}>
          <Text style={styles.demoTitle}>Demo Access</Text>
          <Text style={styles.demoSubtitle}>Try different user roles</Text>
          
          <View style={styles.demoButtons}>
            <TouchableOpacity 
              style={[styles.demoButton, styles.driverButton]}
              onPress={() => handleDemoLogin(UserRole.DRIVER)}
              disabled={isLoading}
            >
              <Ionicons name="person-outline" size={20} color="#fff" />
              <Text style={styles.demoButtonText}>Driver</Text>
            </TouchableOpacity>

            <TouchableOpacity 
              style={[styles.demoButton, styles.technicianButton]}
              onPress={() => handleDemoLogin(UserRole.TECHNICIAN)}
              disabled={isLoading}
            >
              <Ionicons name="construct-outline" size={20} color="#fff" />
              <Text style={styles.demoButtonText}>Technician</Text>
            </TouchableOpacity>

            <TouchableOpacity 
              style={[styles.demoButton, styles.managerButton]}
              onPress={() => handleDemoLogin(UserRole.MANAGER)}
              disabled={isLoading}
            >
              <Ionicons name="business-outline" size={20} color="#fff" />
              <Text style={styles.demoButtonText}>Manager</Text>
            </TouchableOpacity>
          </View>
        </View>

        {/* Footer */}
        <View style={styles.footer}>
          <Text style={styles.footerText}>
            AtlasMesh Fleet OS v2.0.0
          </Text>
          <Text style={styles.footerText}>
            Abu Dhabi, UAE
          </Text>
        </View>
      </ScrollView>
    </KeyboardAvoidingView>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#f8fafc',
  },
  scrollContainer: {
    flexGrow: 1,
    justifyContent: 'center',
    padding: 20,
  },
  header: {
    alignItems: 'center',
    marginBottom: 40,
  },
  logo: {
    width: 120,
    height: 120,
    borderRadius: 60,
    marginBottom: 20,
  },
  title: {
    fontSize: 28,
    fontWeight: 'bold',
    color: theme.colors.primary,
    marginBottom: 8,
  },
  subtitle: {
    fontSize: 16,
    color: '#666',
    textAlign: 'center',
  },
  formContainer: {
    backgroundColor: '#fff',
    borderRadius: 12,
    padding: 24,
    marginBottom: 24,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.1,
    shadowRadius: 8,
    elevation: 4,
  },
  formTitle: {
    fontSize: 24,
    fontWeight: 'bold',
    color: '#333',
    marginBottom: 24,
    textAlign: 'center',
  },
  inputContainer: {
    flexDirection: 'row',
    alignItems: 'center',
    borderWidth: 1,
    borderColor: '#e2e8f0',
    borderRadius: 8,
    marginBottom: 16,
    paddingHorizontal: 12,
    backgroundColor: '#f8fafc',
  },
  inputIcon: {
    marginRight: 12,
  },
  input: {
    flex: 1,
    height: 48,
    fontSize: 16,
    color: '#333',
  },
  eyeIcon: {
    padding: 8,
  },
  loginButton: {
    backgroundColor: theme.colors.primary,
    borderRadius: 8,
    height: 48,
    justifyContent: 'center',
    alignItems: 'center',
    marginTop: 8,
  },
  loginButtonDisabled: {
    backgroundColor: '#94a3b8',
  },
  loginButtonText: {
    color: '#fff',
    fontSize: 16,
    fontWeight: 'bold',
  },
  forgotPassword: {
    alignItems: 'center',
    marginTop: 16,
  },
  forgotPasswordText: {
    color: theme.colors.primary,
    fontSize: 14,
  },
  demoContainer: {
    backgroundColor: '#fff',
    borderRadius: 12,
    padding: 24,
    marginBottom: 24,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.1,
    shadowRadius: 8,
    elevation: 4,
  },
  demoTitle: {
    fontSize: 20,
    fontWeight: 'bold',
    color: '#333',
    marginBottom: 8,
    textAlign: 'center',
  },
  demoSubtitle: {
    fontSize: 14,
    color: '#666',
    textAlign: 'center',
    marginBottom: 20,
  },
  demoButtons: {
    flexDirection: 'row',
    justifyContent: 'space-between',
  },
  demoButton: {
    flex: 1,
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'center',
    paddingVertical: 12,
    paddingHorizontal: 16,
    borderRadius: 8,
    marginHorizontal: 4,
  },
  driverButton: {
    backgroundColor: '#10b981',
  },
  technicianButton: {
    backgroundColor: '#f59e0b',
  },
  managerButton: {
    backgroundColor: '#8b5cf6',
  },
  demoButtonText: {
    color: '#fff',
    fontSize: 14,
    fontWeight: 'bold',
    marginLeft: 8,
  },
  footer: {
    alignItems: 'center',
    marginTop: 20,
  },
  footerText: {
    fontSize: 12,
    color: '#999',
    marginBottom: 4,
  },
});
