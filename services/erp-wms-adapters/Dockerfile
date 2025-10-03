# ERP/WMS Adapters Service
FROM golang:1.21-alpine AS builder

WORKDIR /app
COPY go.mod go.sum ./
RUN go mod download

COPY . .
RUN CGO_ENABLED=0 GOOS=linux go build -o erp-wms-adapters ./cmd

FROM alpine:3.18
RUN apk --no-cache add ca-certificates tzdata
WORKDIR /root/

COPY --from=builder /app/erp-wms-adapters .

EXPOSE 8080
CMD ["./erp-wms-adapters"]
