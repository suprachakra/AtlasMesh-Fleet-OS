#!/bin/bash
# AtlasMesh Fleet OS - Performance Budget Enforcement Script

set -e

# Performance budgets (in milliseconds)
CONTROL_LOOP_P95_BUDGET=100
CONTROL_LOOP_P99_BUDGET=200
POLICY_EVAL_P95_BUDGET=50
POLICY_EVAL_P99_BUDGET=100
ROUTE_CALC_P95_BUDGET=500
ROUTE_CALC_P99_BUDGET=1000

# API endpoints for metrics
PROMETHEUS_URL="http://localhost:9090"
GRAFANA_URL="http://localhost:3000"

echo "AtlasMesh Fleet OS - Performance Budget Check"
echo "============================================="

# Function to query Prometheus metrics
query_prometheus() {
    local query=$1
    local result
    
    result=$(curl -s -G "$PROMETHEUS_URL/api/v1/query" \
        --data-urlencode "query=$query" | \
        jq -r '.data.result[0].value[1] // "0"')
    
    echo "$result"
}

# Function to check budget compliance
check_budget() {
    local metric_name=$1
    local current_value=$2
    local budget=$3
    local unit=$4
    
    echo -n "Checking $metric_name budget... "
    
    if (( $(echo "$current_value <= $budget" | bc -l) )); then
        echo "✅ PASS (${current_value}${unit} <= ${budget}${unit})"
        return 0
    else
        echo "❌ FAIL (${current_value}${unit} > ${budget}${unit})"
        return 1
    fi
}

# Check control loop latency
echo "Control Loop Performance:"
control_loop_p95=$(query_prometheus 'histogram_quantile(0.95, rate(atlasmesh_control_loop_duration_seconds_bucket[5m])) * 1000')
control_loop_p99=$(query_prometheus 'histogram_quantile(0.99, rate(atlasmesh_control_loop_duration_seconds_bucket[5m])) * 1000')

check_budget "Control Loop P95" "$control_loop_p95" "$CONTROL_LOOP_P95_BUDGET" "ms"
control_loop_p95_pass=$?

check_budget "Control Loop P99" "$control_loop_p99" "$CONTROL_LOOP_P99_BUDGET" "ms"
control_loop_p99_pass=$?

# Check policy evaluation latency
echo -e "\nPolicy Evaluation Performance:"
policy_eval_p95=$(query_prometheus 'histogram_quantile(0.95, rate(atlasmesh_policy_evaluation_duration_seconds_bucket[5m])) * 1000')
policy_eval_p99=$(query_prometheus 'histogram_quantile(0.99, rate(atlasmesh_policy_evaluation_duration_seconds_bucket[5m])) * 1000')

check_budget "Policy Eval P95" "$policy_eval_p95" "$POLICY_EVAL_P95_BUDGET" "ms"
policy_eval_p95_pass=$?

check_budget "Policy Eval P99" "$policy_eval_p99" "$POLICY_EVAL_P99_BUDGET" "ms"
policy_eval_p99_pass=$?

# Check route calculation latency
echo -e "\nRoute Calculation Performance:"
route_calc_p95=$(query_prometheus 'histogram_quantile(0.95, rate(atlasmesh_route_calculation_duration_seconds_bucket[5m])) * 1000')
route_calc_p99=$(query_prometheus 'histogram_quantile(0.99, rate(atlasmesh_route_calculation_duration_seconds_bucket[5m])) * 1000')

check_budget "Route Calc P95" "$route_calc_p95" "$ROUTE_CALC_P95_BUDGET" "ms"
route_calc_p95_pass=$?

check_budget "Route Calc P99" "$route_calc_p99" "$ROUTE_CALC_P99_BUDGET" "ms"
route_calc_p99_pass=$?

# Calculate overall pass/fail
total_checks=6
failed_checks=$((control_loop_p95_pass + control_loop_p99_pass + policy_eval_p95_pass + policy_eval_p99_pass + route_calc_p95_pass + route_calc_p99_pass))
passed_checks=$((total_checks - failed_checks))

echo -e "\nPerformance Budget Summary:"
echo "=========================="
echo "Passed: $passed_checks/$total_checks"
echo "Failed: $failed_checks/$total_checks"

if [ $failed_checks -eq 0 ]; then
    echo "✅ All performance budgets met!"
    exit 0
else
    echo "❌ Performance budget violations detected!"
    echo "Please optimize the failing components before deployment."
    exit 1
fi
