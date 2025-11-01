package main

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/gorilla/mux"
	"github.com/prometheus/client_golang/prometheus/promhttp"
)

// ERP/WMS Adapters Service
// Handles integration with SAP, Oracle WMS, IBM Maximo, Salesforce CRM

type ERPWMSAdaptersService struct {
	sapAdapter       *SAPAdapter
	oracleAdapter    *OracleWMSAdapter
	maximoAdapter    *IBMMaximoAdapter
	salesforceAdapter *SalesforceAdapter
}

// SAP Integration
type SAPAdapter struct {
	baseURL    string
	username   string
	password   string
	client     string
	httpClient *http.Client
}

type SAPOrder struct {
	OrderID       string    `json:"order_id"`
	CustomerID    string    `json:"customer_id"`
	OrderType     string    `json:"order_type"`
	Status        string    `json:"status"`
	Priority      int       `json:"priority"`
	CreatedDate   time.Time `json:"created_date"`
	RequestedDate time.Time `json:"requested_date"`
	Items         []SAPOrderItem `json:"items"`
	ShippingInfo  SAPShippingInfo `json:"shipping_info"`
	BillingInfo   SAPBillingInfo `json:"billing_info"`
	TotalValue    float64   `json:"total_value"`
	Currency      string    `json:"currency"`
}

type SAPOrderItem struct {
	ItemID      string  `json:"item_id"`
	ProductID   string  `json:"product_id"`
	Description string  `json:"description"`
	Quantity    int     `json:"quantity"`
	UnitPrice   float64 `json:"unit_price"`
	TotalPrice  float64 `json:"total_price"`
	Weight      float64 `json:"weight_kg"`
	Dimensions  SAPDimensions `json:"dimensions"`
}

type SAPDimensions struct {
	Length float64 `json:"length_cm"`
	Width  float64 `json:"width_cm"`
	Height float64 `json:"height_cm"`
}

type SAPShippingInfo struct {
	Address     string    `json:"address"`
	City        string    `json:"city"`
	PostalCode  string    `json:"postal_code"`
	Country     string    `json:"country"`
	ContactName string    `json:"contact_name"`
	Phone       string    `json:"phone"`
	Email       string    `json:"email"`
	Instructions string   `json:"special_instructions"`
	TimeWindow  SAPTimeWindow `json:"delivery_window"`
}

type SAPTimeWindow struct {
	StartTime time.Time `json:"start_time"`
	EndTime   time.Time `json:"end_time"`
	Timezone  string    `json:"timezone"`
}

type SAPBillingInfo struct {
	BillingAddress string `json:"billing_address"`
	PaymentMethod  string `json:"payment_method"`
	PaymentTerms   string `json:"payment_terms"`
	TaxID          string `json:"tax_id"`
}

// Oracle WMS Integration
type OracleWMSAdapter struct {
	baseURL    string
	username   string
	password   string
	httpClient *http.Client
}

type OracleInventory struct {
	ItemID         string    `json:"item_id"`
	SKU            string    `json:"sku"`
	Description    string    `json:"description"`
	Category       string    `json:"category"`
	Location       string    `json:"warehouse_location"`
	Zone           string    `json:"zone"`
	AvailableQty   int       `json:"available_quantity"`
	ReservedQty    int       `json:"reserved_quantity"`
	OnOrderQty     int       `json:"on_order_quantity"`
	UnitCost       float64   `json:"unit_cost"`
	LastMovement   time.Time `json:"last_movement"`
	ExpiryDate     *time.Time `json:"expiry_date,omitempty"`
	SerialNumbers  []string  `json:"serial_numbers"`
	BatchNumbers   []string  `json:"batch_numbers"`
	Attributes     map[string]interface{} `json:"attributes"`
}

type OraclePickList struct {
	PickListID   string    `json:"pick_list_id"`
	OrderID      string    `json:"order_id"`
	Status       string    `json:"status"`
	Priority     int       `json:"priority"`
	CreatedDate  time.Time `json:"created_date"`
	AssignedTo   string    `json:"assigned_to"`
	Items        []OraclePickItem `json:"items"`
	Instructions []string  `json:"instructions"`
}

type OraclePickItem struct {
	ItemID       string  `json:"item_id"`
	SKU          string  `json:"sku"`
	Quantity     int     `json:"quantity"`
	Location     string  `json:"location"`
	Zone         string  `json:"zone"`
	SerialNumber string  `json:"serial_number,omitempty"`
	BatchNumber  string  `json:"batch_number,omitempty"`
	Status       string  `json:"status"`
}

// IBM Maximo Integration
type IBMMaximoAdapter struct {
	baseURL    string
	apiKey     string
	httpClient *http.Client
}

type MaximoWorkOrder struct {
	WorkOrderID   string    `json:"work_order_id"`
	AssetID       string    `json:"asset_id"`
	AssetType     string    `json:"asset_type"`
	Description   string    `json:"description"`
	Priority      int       `json:"priority"`
	Status        string    `json:"status"`
	WorkType      string    `json:"work_type"`
	CreatedDate   time.Time `json:"created_date"`
	ScheduledDate *time.Time `json:"scheduled_date,omitempty"`
	CompletedDate *time.Time `json:"completed_date,omitempty"`
	AssignedTo    string    `json:"assigned_to"`
	Location      string    `json:"location"`
	Tasks         []MaximoTask `json:"tasks"`
	Materials     []MaximoMaterial `json:"materials"`
	Labor         []MaximoLabor `json:"labor"`
	TotalCost     float64   `json:"total_cost"`
}

type MaximoTask struct {
	TaskID      string  `json:"task_id"`
	Description string  `json:"description"`
	Status      string  `json:"status"`
	Duration    int     `json:"duration_minutes"`
	Skills      []string `json:"required_skills"`
}

type MaximoMaterial struct {
	MaterialID  string  `json:"material_id"`
	Description string  `json:"description"`
	Quantity    int     `json:"quantity"`
	UnitCost    float64 `json:"unit_cost"`
	TotalCost   float64 `json:"total_cost"`
}

type MaximoLabor struct {
	PersonID    string  `json:"person_id"`
	Name        string  `json:"name"`
	Skills      []string `json:"skills"`
	HourlyRate  float64 `json:"hourly_rate"`
	Hours       float64 `json:"hours"`
	TotalCost   float64 `json:"total_cost"`
}

type MaximoAsset struct {
	AssetID       string    `json:"asset_id"`
	AssetNumber   string    `json:"asset_number"`
	Description   string    `json:"description"`
	AssetType     string    `json:"asset_type"`
	Status        string    `json:"status"`
	Location      string    `json:"location"`
	Manufacturer  string    `json:"manufacturer"`
	Model         string    `json:"model"`
	SerialNumber  string    `json:"serial_number"`
	InstallDate   time.Time `json:"install_date"`
	WarrantyEnd   *time.Time `json:"warranty_end,omitempty"`
	LastMaintenance *time.Time `json:"last_maintenance,omitempty"`
	NextMaintenance *time.Time `json:"next_maintenance,omitempty"`
	Specifications map[string]interface{} `json:"specifications"`
}

// Salesforce Integration
type SalesforceAdapter struct {
	instanceURL  string
	accessToken  string
	clientID     string
	clientSecret string
	httpClient   *http.Client
}

type SalesforceAccount struct {
	AccountID   string    `json:"account_id"`
	Name        string    `json:"name"`
	Type        string    `json:"type"`
	Industry    string    `json:"industry"`
	Status      string    `json:"status"`
	Phone       string    `json:"phone"`
	Email       string    `json:"email"`
	Website     string    `json:"website"`
	Address     SalesforceAddress `json:"address"`
	CreatedDate time.Time `json:"created_date"`
	LastModified time.Time `json:"last_modified"`
	Owner       string    `json:"owner"`
}

type SalesforceAddress struct {
	Street     string `json:"street"`
	City       string `json:"city"`
	State      string `json:"state"`
	PostalCode string `json:"postal_code"`
	Country    string `json:"country"`
}

type SalesforceOpportunity struct {
	OpportunityID string    `json:"opportunity_id"`
	AccountID     string    `json:"account_id"`
	Name          string    `json:"name"`
	Stage         string    `json:"stage"`
	Amount        float64   `json:"amount"`
	Currency      string    `json:"currency"`
	Probability   int       `json:"probability"`
	CloseDate     time.Time `json:"close_date"`
	Owner         string    `json:"owner"`
	Description   string    `json:"description"`
	NextStep      string    `json:"next_step"`
	LeadSource    string    `json:"lead_source"`
}

type SalesforceCase struct {
	CaseID      string    `json:"case_id"`
	AccountID   string    `json:"account_id"`
	ContactID   string    `json:"contact_id"`
	Subject     string    `json:"subject"`
	Description string    `json:"description"`
	Status      string    `json:"status"`
	Priority    string    `json:"priority"`
	Origin      string    `json:"origin"`
	Type        string    `json:"type"`
	CreatedDate time.Time `json:"created_date"`
	Owner       string    `json:"owner"`
}

func main() {
	// Initialize service
	service := &ERPWMSAdaptersService{
		sapAdapter:        initSAPAdapter(),
		oracleAdapter:     initOracleWMSAdapter(),
		maximoAdapter:     initIBMMaximoAdapter(),
		salesforceAdapter: initSalesforceAdapter(),
	}

	// Setup router
	router := mux.NewRouter()
	
	// SAP endpoints
	router.HandleFunc("/api/v1/sap/orders", service.getSAPOrders).Methods("GET")
	router.HandleFunc("/api/v1/sap/orders", service.createSAPOrder).Methods("POST")
	router.HandleFunc("/api/v1/sap/orders/{order_id}", service.updateSAPOrder).Methods("PUT")
	router.HandleFunc("/api/v1/sap/orders/{order_id}/status", service.getSAPOrderStatus).Methods("GET")
	
	// Oracle WMS endpoints
	router.HandleFunc("/api/v1/oracle/inventory", service.getOracleInventory).Methods("GET")
	router.HandleFunc("/api/v1/oracle/inventory/{item_id}", service.updateOracleInventory).Methods("PUT")
	router.HandleFunc("/api/v1/oracle/picklists", service.getOraclePickLists).Methods("GET")
	router.HandleFunc("/api/v1/oracle/picklists", service.createOraclePickList).Methods("POST")
	
	// IBM Maximo endpoints
	router.HandleFunc("/api/v1/maximo/workorders", service.getMaximoWorkOrders).Methods("GET")
	router.HandleFunc("/api/v1/maximo/workorders", service.createMaximoWorkOrder).Methods("POST")
	router.HandleFunc("/api/v1/maximo/workorders/{work_order_id}", service.updateMaximoWorkOrder).Methods("PUT")
	router.HandleFunc("/api/v1/maximo/assets", service.getMaximoAssets).Methods("GET")
	router.HandleFunc("/api/v1/maximo/assets/{asset_id}", service.getMaximoAsset).Methods("GET")
	
	// Salesforce endpoints
	router.HandleFunc("/api/v1/salesforce/accounts", service.getSalesforceAccounts).Methods("GET")
	router.HandleFunc("/api/v1/salesforce/accounts", service.createSalesforceAccount).Methods("POST")
	router.HandleFunc("/api/v1/salesforce/opportunities", service.getSalesforceOpportunities).Methods("GET")
	router.HandleFunc("/api/v1/salesforce/cases", service.getSalesforceCases).Methods("GET")
	router.HandleFunc("/api/v1/salesforce/cases", service.createSalesforceCase).Methods("POST")
	
	// Health and metrics
	router.HandleFunc("/health", service.healthCheck).Methods("GET")
	router.Handle("/metrics", promhttp.Handler()).Methods("GET")

	// Start server
	server := &http.Server{
		Addr:         ":8080",
		Handler:      router,
		ReadTimeout:  30 * time.Second,
		WriteTimeout: 30 * time.Second,
		IdleTimeout:  60 * time.Second,
	}

	// Graceful shutdown
	go func() {
		log.Println("🚀 ERP/WMS Adapters Service starting on :8080")
		if err := server.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("❌ Server failed to start: %v", err)
		}
	}()

	// Wait for interrupt signal
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	log.Println("🛑 Shutting down ERP/WMS Adapters Service...")
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()

	if err := server.Shutdown(ctx); err != nil {
		log.Fatalf("❌ Server forced to shutdown: %v", err)
	}

	log.Println("✅ ERP/WMS Adapters Service stopped")
}

func initSAPAdapter() *SAPAdapter {
	return &SAPAdapter{
		baseURL:  getEnv("SAP_BASE_URL", "https://sap.company.com"),
		username: getEnv("SAP_USERNAME", ""),
		password: getEnv("SAP_PASSWORD", ""),
		client:   getEnv("SAP_CLIENT", "100"),
		httpClient: &http.Client{Timeout: 30 * time.Second},
	}
}

func initOracleWMSAdapter() *OracleWMSAdapter {
	return &OracleWMSAdapter{
		baseURL:  getEnv("ORACLE_WMS_URL", "https://oracle-wms.company.com"),
		username: getEnv("ORACLE_WMS_USERNAME", ""),
		password: getEnv("ORACLE_WMS_PASSWORD", ""),
		httpClient: &http.Client{Timeout: 30 * time.Second},
	}
}

func initIBMMaximoAdapter() *IBMMaximoAdapter {
	return &IBMMaximoAdapter{
		baseURL:  getEnv("MAXIMO_BASE_URL", "https://maximo.company.com"),
		apiKey:   getEnv("MAXIMO_API_KEY", ""),
		httpClient: &http.Client{Timeout: 30 * time.Second},
	}
}

func initSalesforceAdapter() *SalesforceAdapter {
	return &SalesforceAdapter{
		instanceURL:  getEnv("SALESFORCE_INSTANCE_URL", "https://company.salesforce.com"),
		clientID:     getEnv("SALESFORCE_CLIENT_ID", ""),
		clientSecret: getEnv("SALESFORCE_CLIENT_SECRET", ""),
		httpClient:   &http.Client{Timeout: 30 * time.Second},
	}
}

// SAP Integration Methods
func (s *ERPWMSAdaptersService) getSAPOrders(w http.ResponseWriter, r *http.Request) {
	// Mock SAP orders data
	orders := []SAPOrder{
		{
			OrderID:     "SAP-001",
			CustomerID:  "CUST-001",
			OrderType:   "delivery",
			Status:      "confirmed",
			Priority:    1,
			CreatedDate: time.Now().Add(-2 * time.Hour),
			RequestedDate: time.Now().Add(4 * time.Hour),
			Items: []SAPOrderItem{
				{
					ItemID:      "ITEM-001",
					ProductID:   "PROD-001",
					Description: "Electronics Package",
					Quantity:    2,
					UnitPrice:   150.0,
					TotalPrice:  300.0,
					Weight:      5.5,
					Dimensions: SAPDimensions{
						Length: 30.0,
						Width:  20.0,
						Height: 15.0,
					},
				},
			},
			ShippingInfo: SAPShippingInfo{
				Address:     "123 Business Bay",
				City:        "Dubai",
				PostalCode:  "12345",
				Country:     "UAE",
				ContactName: "Ahmed Al-Mansouri",
				Phone:       "+971501234567",
				Email:       "ahmed@company.ae",
				TimeWindow: SAPTimeWindow{
					StartTime: time.Now().Add(4 * time.Hour),
					EndTime:   time.Now().Add(8 * time.Hour),
					Timezone:  "Asia/Dubai",
				},
			},
			TotalValue: 300.0,
			Currency:   "AED",
		},
	}

	log.Printf("✅ Retrieved %d SAP orders", len(orders))
	s.sendJSON(w, http.StatusOK, orders)
}

func (s *ERPWMSAdaptersService) createSAPOrder(w http.ResponseWriter, r *http.Request) {
	var order SAPOrder
	if err := json.NewDecoder(r.Body).Decode(&order); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	// Generate order ID and set defaults
	order.OrderID = fmt.Sprintf("SAP-%d", time.Now().Unix())
	order.CreatedDate = time.Now()
	order.Status = "pending"

	// Mock SAP order creation
	log.Printf("✅ SAP order created: %s", order.OrderID)
	s.sendJSON(w, http.StatusCreated, order)
}

func (s *ERPWMSAdaptersService) updateSAPOrder(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	orderID := vars["order_id"]

	var updates map[string]interface{}
	if err := json.NewDecoder(r.Body).Decode(&updates); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	// Mock SAP order update
	response := map[string]interface{}{
		"order_id":     orderID,
		"status":       "updated",
		"last_updated": time.Now(),
		"updates":      updates,
	}

	log.Printf("✅ SAP order updated: %s", orderID)
	s.sendJSON(w, http.StatusOK, response)
}

func (s *ERPWMSAdaptersService) getSAPOrderStatus(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	orderID := vars["order_id"]

	// Mock SAP order status
	status := map[string]interface{}{
		"order_id":     orderID,
		"status":       "in_transit",
		"last_updated": time.Now(),
		"tracking_id":  fmt.Sprintf("TRK-%d", time.Now().Unix()),
	}

	log.Printf("✅ SAP order status retrieved: %s", orderID)
	s.sendJSON(w, http.StatusOK, status)
}

// Oracle WMS Integration Methods
func (s *ERPWMSAdaptersService) getOracleInventory(w http.ResponseWriter, r *http.Request) {
	// Mock Oracle inventory data
	inventory := []OracleInventory{
		{
			ItemID:       "ITEM-001",
			SKU:          "SKU-001",
			Description:  "Electronics Package",
			Category:     "Electronics",
			Location:     "WH-A-001",
			Zone:         "Zone-A",
			AvailableQty: 50,
			ReservedQty:  10,
			OnOrderQty:   20,
			UnitCost:     120.0,
			LastMovement: time.Now().Add(-1 * time.Hour),
		},
	}

	log.Printf("✅ Retrieved %d Oracle inventory items", len(inventory))
	s.sendJSON(w, http.StatusOK, inventory)
}

func (s *ERPWMSAdaptersService) updateOracleInventory(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	itemID := vars["item_id"]

	var updates map[string]interface{}
	if err := json.NewDecoder(r.Body).Decode(&updates); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	// Mock Oracle inventory update
	response := map[string]interface{}{
		"item_id":      itemID,
		"status":       "updated",
		"last_updated": time.Now(),
		"updates":      updates,
	}

	log.Printf("✅ Oracle inventory updated: %s", itemID)
	s.sendJSON(w, http.StatusOK, response)
}

func (s *ERPWMSAdaptersService) getOraclePickLists(w http.ResponseWriter, r *http.Request) {
	// Mock Oracle pick lists
	pickLists := []OraclePickList{
		{
			PickListID:  "PL-001",
			OrderID:     "SAP-001",
			Status:      "assigned",
			Priority:    1,
			CreatedDate: time.Now().Add(-30 * time.Minute),
			AssignedTo:  "picker-001",
			Items: []OraclePickItem{
				{
					ItemID:   "ITEM-001",
					SKU:      "SKU-001",
					Quantity: 2,
					Location: "WH-A-001",
					Zone:     "Zone-A",
					Status:   "pending",
				},
			},
		},
	}

	log.Printf("✅ Retrieved %d Oracle pick lists", len(pickLists))
	s.sendJSON(w, http.StatusOK, pickLists)
}

func (s *ERPWMSAdaptersService) createOraclePickList(w http.ResponseWriter, r *http.Request) {
	var pickList OraclePickList
	if err := json.NewDecoder(r.Body).Decode(&pickList); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	// Generate pick list ID and set defaults
	pickList.PickListID = fmt.Sprintf("PL-%d", time.Now().Unix())
	pickList.CreatedDate = time.Now()
	pickList.Status = "created"

	log.Printf("✅ Oracle pick list created: %s", pickList.PickListID)
	s.sendJSON(w, http.StatusCreated, pickList)
}

// IBM Maximo Integration Methods
func (s *ERPWMSAdaptersService) getMaximoWorkOrders(w http.ResponseWriter, r *http.Request) {
	// Mock Maximo work orders
	workOrders := []MaximoWorkOrder{
		{
			WorkOrderID: "WO-001",
			AssetID:     "VEHICLE-001",
			AssetType:   "autonomous_vehicle",
			Description: "Scheduled maintenance",
			Priority:    2,
			Status:      "approved",
			WorkType:    "preventive",
			CreatedDate: time.Now().Add(-1 * time.Hour),
			ScheduledDate: func() *time.Time { t := time.Now().Add(24 * time.Hour); return &t }(),
			AssignedTo:  "tech-001",
			Location:    "Garage-A",
			Tasks: []MaximoTask{
				{
					TaskID:      "TASK-001",
					Description: "Oil change",
					Status:      "pending",
					Duration:    60,
					Skills:      []string{"mechanical", "automotive"},
				},
			},
			Materials: []MaximoMaterial{
				{
					MaterialID:  "OIL-001",
					Description: "Engine Oil 5W-30",
					Quantity:    5,
					UnitCost:    25.0,
					TotalCost:   125.0,
				},
			},
			TotalCost: 225.0,
		},
	}

	log.Printf("✅ Retrieved %d Maximo work orders", len(workOrders))
	s.sendJSON(w, http.StatusOK, workOrders)
}

func (s *ERPWMSAdaptersService) createMaximoWorkOrder(w http.ResponseWriter, r *http.Request) {
	var workOrder MaximoWorkOrder
	if err := json.NewDecoder(r.Body).Decode(&workOrder); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	// Generate work order ID and set defaults
	workOrder.WorkOrderID = fmt.Sprintf("WO-%d", time.Now().Unix())
	workOrder.CreatedDate = time.Now()
	workOrder.Status = "created"

	log.Printf("✅ Maximo work order created: %s", workOrder.WorkOrderID)
	s.sendJSON(w, http.StatusCreated, workOrder)
}

func (s *ERPWMSAdaptersService) updateMaximoWorkOrder(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	workOrderID := vars["work_order_id"]

	var updates map[string]interface{}
	if err := json.NewDecoder(r.Body).Decode(&updates); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	// Mock Maximo work order update
	response := map[string]interface{}{
		"work_order_id": workOrderID,
		"status":        "updated",
		"last_updated":  time.Now(),
		"updates":       updates,
	}

	log.Printf("✅ Maximo work order updated: %s", workOrderID)
	s.sendJSON(w, http.StatusOK, response)
}

func (s *ERPWMSAdaptersService) getMaximoAssets(w http.ResponseWriter, r *http.Request) {
	// Mock Maximo assets
	assets := []MaximoAsset{
		{
			AssetID:      "VEHICLE-001",
			AssetNumber:  "AV-001",
			Description:  "Autonomous Vehicle Model X",
			AssetType:    "autonomous_vehicle",
			Status:       "operational",
			Location:     "Fleet-A",
			Manufacturer: "TechCorp",
			Model:        "AV-X1",
			SerialNumber: "SN123456",
			InstallDate:  time.Now().Add(-365 * 24 * time.Hour),
			WarrantyEnd:  func() *time.Time { t := time.Now().Add(730 * 24 * time.Hour); return &t }(),
			LastMaintenance: func() *time.Time { t := time.Now().Add(-30 * 24 * time.Hour); return &t }(),
			NextMaintenance: func() *time.Time { t := time.Now().Add(30 * 24 * time.Hour); return &t }(),
			Specifications: map[string]interface{}{
				"max_speed":    "60 km/h",
				"battery_type": "Lithium-ion",
				"capacity":     "8 passengers",
			},
		},
	}

	log.Printf("✅ Retrieved %d Maximo assets", len(assets))
	s.sendJSON(w, http.StatusOK, assets)
}

func (s *ERPWMSAdaptersService) getMaximoAsset(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	assetID := vars["asset_id"]

	// Mock single Maximo asset
	asset := MaximoAsset{
		AssetID:      assetID,
		AssetNumber:  "AV-001",
		Description:  "Autonomous Vehicle Model X",
		AssetType:    "autonomous_vehicle",
		Status:       "operational",
		Location:     "Fleet-A",
		Manufacturer: "TechCorp",
		Model:        "AV-X1",
		SerialNumber: "SN123456",
		InstallDate:  time.Now().Add(-365 * 24 * time.Hour),
	}

	log.Printf("✅ Retrieved Maximo asset: %s", assetID)
	s.sendJSON(w, http.StatusOK, asset)
}

// Salesforce Integration Methods
func (s *ERPWMSAdaptersService) getSalesforceAccounts(w http.ResponseWriter, r *http.Request) {
	// Mock Salesforce accounts
	accounts := []SalesforceAccount{
		{
			AccountID: "ACC-001",
			Name:      "Emirates Transport",
			Type:      "Customer",
			Industry:  "Transportation",
			Status:    "Active",
			Phone:     "+971501234567",
			Email:     "contact@emirates-transport.ae",
			Address: SalesforceAddress{
				Street:     "Sheikh Zayed Road",
				City:       "Dubai",
				PostalCode: "12345",
				Country:    "UAE",
			},
			CreatedDate:  time.Now().Add(-30 * 24 * time.Hour),
			LastModified: time.Now().Add(-1 * time.Hour),
			Owner:        "sales-rep-001",
		},
	}

	log.Printf("✅ Retrieved %d Salesforce accounts", len(accounts))
	s.sendJSON(w, http.StatusOK, accounts)
}

func (s *ERPWMSAdaptersService) createSalesforceAccount(w http.ResponseWriter, r *http.Request) {
	var account SalesforceAccount
	if err := json.NewDecoder(r.Body).Decode(&account); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	// Generate account ID and set defaults
	account.AccountID = fmt.Sprintf("ACC-%d", time.Now().Unix())
	account.CreatedDate = time.Now()
	account.LastModified = time.Now()
	account.Status = "Active"

	log.Printf("✅ Salesforce account created: %s", account.AccountID)
	s.sendJSON(w, http.StatusCreated, account)
}

func (s *ERPWMSAdaptersService) getSalesforceOpportunities(w http.ResponseWriter, r *http.Request) {
	// Mock Salesforce opportunities
	opportunities := []SalesforceOpportunity{
		{
			OpportunityID: "OPP-001",
			AccountID:     "ACC-001",
			Name:          "Fleet Expansion Project",
			Stage:         "Proposal/Quote",
			Amount:        500000.0,
			Currency:      "AED",
			Probability:   75,
			CloseDate:     time.Now().Add(30 * 24 * time.Hour),
			Owner:         "sales-rep-001",
			Description:   "Expansion of autonomous vehicle fleet",
			NextStep:      "Final presentation",
			LeadSource:    "Website",
		},
	}

	log.Printf("✅ Retrieved %d Salesforce opportunities", len(opportunities))
	s.sendJSON(w, http.StatusOK, opportunities)
}

func (s *ERPWMSAdaptersService) getSalesforceCases(w http.ResponseWriter, r *http.Request) {
	// Mock Salesforce cases
	cases := []SalesforceCase{
		{
			CaseID:      "CASE-001",
			AccountID:   "ACC-001",
			ContactID:   "CONT-001",
			Subject:     "Vehicle maintenance inquiry",
			Description: "Customer inquiry about scheduled maintenance",
			Status:      "Open",
			Priority:    "Medium",
			Origin:      "Phone",
			Type:        "Question",
			CreatedDate: time.Now().Add(-2 * time.Hour),
			Owner:       "support-rep-001",
		},
	}

	log.Printf("✅ Retrieved %d Salesforce cases", len(cases))
	s.sendJSON(w, http.StatusOK, cases)
}

func (s *ERPWMSAdaptersService) createSalesforceCase(w http.ResponseWriter, r *http.Request) {
	var salesforceCase SalesforceCase
	if err := json.NewDecoder(r.Body).Decode(&salesforceCase); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	// Generate case ID and set defaults
	salesforceCase.CaseID = fmt.Sprintf("CASE-%d", time.Now().Unix())
	salesforceCase.CreatedDate = time.Now()
	salesforceCase.Status = "New"

	log.Printf("✅ Salesforce case created: %s", salesforceCase.CaseID)
	s.sendJSON(w, http.StatusCreated, salesforceCase)
}

// Utility Methods
func (s *ERPWMSAdaptersService) healthCheck(w http.ResponseWriter, r *http.Request) {
	health := map[string]interface{}{
		"status":    "healthy",
		"timestamp": time.Now(),
		"adapters": map[string]string{
			"sap":        "connected",
			"oracle_wms": "connected",
			"maximo":     "connected",
			"salesforce": "connected",
		},
		"version": "1.0.0",
	}

	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	json.NewEncoder(w).Encode(health)
}

func (s *ERPWMSAdaptersService) handleError(w http.ResponseWriter, message string, err error, statusCode int) {
	if err != nil {
		log.Printf("❌ %s: %v", message, err)
	} else {
		log.Printf("❌ %s", message)
	}

	response := map[string]interface{}{
		"error":     message,
		"timestamp": time.Now(),
		"status":    statusCode,
	}

	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(statusCode)
	json.NewEncoder(w).Encode(response)
}

func (s *ERPWMSAdaptersService) sendJSON(w http.ResponseWriter, statusCode int, data interface{}) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(statusCode)
	json.NewEncoder(w).Encode(data)
}

func getEnv(key, defaultValue string) string {
	if value := os.Getenv(key); value != "" {
		return value
	}
	return defaultValue
}
