# Payment Adapter

## Purpose
The Payment Adapter provides integration with payment systems for autonomous vehicle fleet management in ride-hail operations. This adapter enables secure payment processing, fare calculation, and financial reporting for robotaxi services.

## Features
- **Payment Processing**: Integration with major payment providers (Stripe, PayPal, etc.)
- **Fare Calculation**: Dynamic fare calculation based on distance, time, and demand
- **Payment Methods**: Support for credit cards, digital wallets, and mobile payments
- **Refund Processing**: Automated refund processing for cancelled rides
- **Financial Reporting**: Real-time financial reporting and analytics
- **Fraud Detection**: Integration with fraud detection systems

## Architecture
```
┌─────────────────────────────────────────────────────────────┐
│ Payment Adapter                                             │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│ ┌──────────────┐ ┌──────────────┐ ┌─────────────┐          │
│ │ Payment      │ │ Fare         │ │ Refund      │          │
│ │ Processing   │ │ Calculation  │ │ Processing  │          │
│ └──────────────┘ └──────────────┘ └─────────────┘          │
│                                                             │
│ ┌──────────────┐ ┌──────────────┐ ┌─────────────┐          │
│ │ Financial    │ │ Fraud        │ │ Analytics   │          │
│ │ Reporting    │ │ Detection    │ │ Dashboard   │          │
│ └──────────────┘ └──────────────┘ └─────────────┘          │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## Configuration
The adapter supports configuration through YAML files and environment variables:

```yaml
payment:
  enabled: true
  provider: "stripe"
  api_key: "${PAYMENT_API_KEY}"
  webhook_secret: "${PAYMENT_WEBHOOK_SECRET}"
  currency: "USD"
  fare_calculation:
    base_fare: 2.50
    per_mile: 1.50
    per_minute: 0.25
    minimum_fare: 5.00
  refund_policy:
    cancellation_window_minutes: 5
    refund_percentage: 100
  fraud_detection:
    enabled: true
    risk_threshold: 0.7
```

## API Endpoints
- `POST /api/v1/payment/process` - Process payment
- `GET /api/v1/payment/fare` - Calculate fare
- `POST /api/v1/payment/refund` - Process refund
- `GET /api/v1/payment/transactions` - Get transaction history
- `GET /api/v1/payment/analytics` - Get financial analytics

## Security
- All payments are processed using PCI DSS compliant systems
- API keys are encrypted and stored securely
- All transactions are logged for audit purposes
- Fraud detection is enabled by default

## Deployment
The adapter can be deployed using Docker or Kubernetes:

```bash
# Docker
docker build -t payment-adapter .
docker run -p 8080:8080 payment-adapter

# Kubernetes
kubectl apply -f k8s/deployment.yaml
```

## Monitoring
The adapter provides Prometheus metrics for monitoring:
- `payment_requests_total` - Total payment requests
- `payment_success_rate` - Payment success rate
- `payment_amount_total` - Total payment amount
- `refund_requests_total` - Total refund requests

## Testing
Run the test suite:

```bash
go test ./...
```

## License
This adapter is part of the AtlasMesh Fleet OS project and is subject to the project's license terms.
