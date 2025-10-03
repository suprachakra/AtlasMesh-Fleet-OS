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

// Arabic Localization Service
// Handles RTL UI support, Arabic translations, Islamic calendar integration, and UAE cultural adaptations

type ArabicLocalizationService struct {
	translationEngine *TranslationEngine
	calendarService   *IslamicCalendarService
	culturalAdapter   *CulturalAdapter
	rtlProcessor      *RTLProcessor
}

// Translation Engine
type TranslationEngine struct {
	translations map[string]map[string]string // language -> key -> translation
	contexts     map[string]TranslationContext
}

type TranslationContext struct {
	Domain      string                 `json:"domain"`
	Context     string                 `json:"context"`
	Metadata    map[string]interface{} `json:"metadata"`
	Gender      string                 `json:"gender,omitempty"`
	Formality   string                 `json:"formality,omitempty"`
	Regional    string                 `json:"regional_variant,omitempty"`
}

type TranslationRequest struct {
	Keys        []string                   `json:"keys"`
	Language    string                     `json:"language"`
	Context     *TranslationContext        `json:"context,omitempty"`
	Variables   map[string]interface{}     `json:"variables,omitempty"`
	Fallback    string                     `json:"fallback_language,omitempty"`
}

type TranslationResponse struct {
	Translations map[string]string          `json:"translations"`
	Language     string                     `json:"language"`
	Context      *TranslationContext        `json:"context,omitempty"`
	Metadata     map[string]interface{}     `json:"metadata"`
	Timestamp    time.Time                  `json:"timestamp"`
}

// Islamic Calendar Service
type IslamicCalendarService struct {
	calendarData map[string]IslamicDate
	holidays     map[string][]Holiday
	prayerTimes  map[string]PrayerSchedule
}

type IslamicDate struct {
	HijriYear    int    `json:"hijri_year"`
	HijriMonth   int    `json:"hijri_month"`
	HijriDay     int    `json:"hijri_day"`
	MonthName    string `json:"month_name"`
	MonthNameAr  string `json:"month_name_arabic"`
	WeekDay      string `json:"week_day"`
	WeekDayAr    string `json:"week_day_arabic"`
	GregorianDate time.Time `json:"gregorian_date"`
}

type Holiday struct {
	HolidayID     string    `json:"holiday_id"`
	Name          string    `json:"name"`
	NameArabic    string    `json:"name_arabic"`
	Type          string    `json:"type"` // religious, national, cultural
	Date          time.Time `json:"date"`
	HijriDate     IslamicDate `json:"hijri_date"`
	Duration      int       `json:"duration_days"`
	Description   string    `json:"description"`
	DescriptionAr string    `json:"description_arabic"`
	Observances   []string  `json:"observances"`
	WorkingDay    bool      `json:"is_working_day"`
}

type PrayerSchedule struct {
	Date        time.Time   `json:"date"`
	Location    string      `json:"location"`
	Coordinates []float64   `json:"coordinates"`
	Prayers     []PrayerTime `json:"prayers"`
	Sunrise     time.Time   `json:"sunrise"`
	Sunset      time.Time   `json:"sunset"`
	QiblaDirection float64  `json:"qibla_direction"`
}

type PrayerTime struct {
	Name     string    `json:"name"`
	NameAr   string    `json:"name_arabic"`
	Time     time.Time `json:"time"`
	Type     string    `json:"type"` // fard, sunnah, nafl
	Duration int       `json:"duration_minutes"`
}

// Cultural Adapter
type CulturalAdapter struct {
	culturalRules map[string]CulturalRule
	adaptations   map[string]CulturalAdaptation
}

type CulturalRule struct {
	RuleID      string                 `json:"rule_id"`
	Category    string                 `json:"category"`
	Description string                 `json:"description"`
	Context     string                 `json:"context"`
	Rules       map[string]interface{} `json:"rules"`
	Priority    int                    `json:"priority"`
	Active      bool                   `json:"active"`
}

type CulturalAdaptation struct {
	AdaptationID string                 `json:"adaptation_id"`
	Type         string                 `json:"type"` // ui, content, behavior, scheduling
	Target       string                 `json:"target"`
	Modifications map[string]interface{} `json:"modifications"`
	Conditions   []string               `json:"conditions"`
	Active       bool                   `json:"active"`
}

type CulturalContext struct {
	Region       string    `json:"region"`
	Language     string    `json:"language"`
	Religion     string    `json:"religion"`
	Gender       string    `json:"gender,omitempty"`
	Age          int       `json:"age,omitempty"`
	Preferences  []string  `json:"preferences"`
	Timestamp    time.Time `json:"timestamp"`
}

// RTL Processor
type RTLProcessor struct {
	rtlRules    map[string]RTLRule
	processors  map[string]TextProcessor
}

type RTLRule struct {
	RuleID      string   `json:"rule_id"`
	Language    string   `json:"language"`
	Script      string   `json:"script"`
	Direction   string   `json:"direction"` // rtl, ltr, mixed
	Alignment   string   `json:"alignment"`
	Components  []string `json:"components"`
	Exceptions  []string `json:"exceptions"`
	Active      bool     `json:"active"`
}

type TextProcessor struct {
	ProcessorID string                 `json:"processor_id"`
	Type        string                 `json:"type"` // bidi, shaping, font
	Language    string                 `json:"language"`
	Rules       map[string]interface{} `json:"rules"`
	Priority    int                    `json:"priority"`
}

type RTLProcessingRequest struct {
	Text        string                 `json:"text"`
	Language    string                 `json:"language"`
	Context     string                 `json:"context"`
	Component   string                 `json:"component"`
	Options     map[string]interface{} `json:"options"`
}

type RTLProcessingResponse struct {
	ProcessedText string                 `json:"processed_text"`
	Direction     string                 `json:"direction"`
	Alignment     string                 `json:"alignment"`
	Styling       map[string]interface{} `json:"styling"`
	Metadata      map[string]interface{} `json:"metadata"`
	Timestamp     time.Time              `json:"timestamp"`
}

func main() {
	// Initialize service
	service := &ArabicLocalizationService{
		translationEngine: initTranslationEngine(),
		calendarService:   initIslamicCalendarService(),
		culturalAdapter:   initCulturalAdapter(),
		rtlProcessor:      initRTLProcessor(),
	}

	// Setup router
	router := mux.NewRouter()
	
	// Translation endpoints
	router.HandleFunc("/api/v1/translations/translate", service.translateText).Methods("POST")
	router.HandleFunc("/api/v1/translations/languages", service.getSupportedLanguages).Methods("GET")
	router.HandleFunc("/api/v1/translations/keys", service.getTranslationKeys).Methods("GET")
	router.HandleFunc("/api/v1/translations/contexts", service.getTranslationContexts).Methods("GET")
	
	// Islamic Calendar endpoints
	router.HandleFunc("/api/v1/calendar/islamic", service.getIslamicDate).Methods("GET")
	router.HandleFunc("/api/v1/calendar/holidays", service.getHolidays).Methods("GET")
	router.HandleFunc("/api/v1/calendar/prayers", service.getPrayerTimes).Methods("GET")
	router.HandleFunc("/api/v1/calendar/convert", service.convertDate).Methods("POST")
	
	// Cultural Adaptation endpoints
	router.HandleFunc("/api/v1/cultural/adapt", service.applyCulturalAdaptations).Methods("POST")
	router.HandleFunc("/api/v1/cultural/rules", service.getCulturalRules).Methods("GET")
	router.HandleFunc("/api/v1/cultural/context", service.getCulturalContext).Methods("GET")
	
	// RTL Processing endpoints
	router.HandleFunc("/api/v1/rtl/process", service.processRTL).Methods("POST")
	router.HandleFunc("/api/v1/rtl/rules", service.getRTLRules).Methods("GET")
	router.HandleFunc("/api/v1/rtl/validate", service.validateRTL).Methods("POST")
	
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
		log.Println("🚀 Arabic Localization Service starting on :8080")
		if err := server.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("❌ Server failed to start: %v", err)
		}
	}()

	// Wait for interrupt signal
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	log.Println("🛑 Shutting down Arabic Localization Service...")
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()

	if err := server.Shutdown(ctx); err != nil {
		log.Fatalf("❌ Server forced to shutdown: %v", err)
	}

	log.Println("✅ Arabic Localization Service stopped")
}

func initTranslationEngine() *TranslationEngine {
	translations := make(map[string]map[string]string)
	
	// Initialize Arabic translations
	translations["ar"] = map[string]string{
		"welcome":                "مرحباً",
		"dashboard":              "لوحة القيادة",
		"fleet_management":       "إدارة الأسطول",
		"vehicle":                "مركبة",
		"vehicles":               "المركبات",
		"status":                 "الحالة",
		"active":                 "نشط",
		"inactive":               "غير نشط",
		"maintenance":            "الصيانة",
		"route":                  "المسار",
		"driver":                 "السائق",
		"passenger":              "الراكب",
		"location":               "الموقع",
		"destination":            "الوجهة",
		"departure":              "المغادرة",
		"arrival":                "الوصول",
		"schedule":               "الجدولة",
		"booking":                "الحجز",
		"cancel":                 "إلغاء",
		"confirm":                "تأكيد",
		"save":                   "حفظ",
		"edit":                   "تعديل",
		"delete":                 "حذف",
		"search":                 "بحث",
		"filter":                 "تصفية",
		"sort":                   "ترتيب",
		"settings":               "الإعدادات",
		"profile":                "الملف الشخصي",
		"logout":                 "تسجيل الخروج",
		"login":                  "تسجيل الدخول",
		"username":               "اسم المستخدم",
		"password":               "كلمة المرور",
		"email":                  "البريد الإلكتروني",
		"phone":                  "رقم الهاتف",
		"address":                "العنوان",
		"city":                   "المدينة",
		"country":                "البلد",
		"uae":                    "دولة الإمارات العربية المتحدة",
		"dubai":                  "دبي",
		"abu_dhabi":              "أبوظبي",
		"sharjah":                "الشارقة",
		"ajman":                  "عجمان",
		"ras_al_khaimah":         "رأس الخيمة",
		"fujairah":               "الفجيرة",
		"umm_al_quwain":          "أم القيوين",
		"emergency":              "طوارئ",
		"police":                 "الشرطة",
		"ambulance":              "الإسعاف",
		"fire_department":        "الإطفاء",
		"traffic":                "المرور",
		"weather":                "الطقس",
		"temperature":            "درجة الحرارة",
		"humidity":               "الرطوبة",
		"wind":                   "الرياح",
		"rain":                   "المطر",
		"sunny":                  "مشمس",
		"cloudy":                 "غائم",
		"safety":                 "السلامة",
		"security":               "الأمان",
		"compliance":             "الامتثال",
		"regulation":             "اللائحة",
		"license":                "الرخصة",
		"permit":                 "التصريح",
		"inspection":             "التفتيش",
		"audit":                  "المراجعة",
		"report":                 "التقرير",
		"analytics":              "التحليلات",
		"performance":            "الأداء",
		"efficiency":             "الكفاءة",
		"cost":                   "التكلفة",
		"revenue":                "الإيرادات",
		"profit":                 "الربح",
		"fuel":                   "الوقود",
		"energy":                 "الطاقة",
		"battery":                "البطارية",
		"charging":               "الشحن",
		"electric":               "كهربائي",
		"autonomous":             "ذاتي القيادة",
		"manual":                 "يدوي",
		"automatic":              "تلقائي",
		"speed":                  "السرعة",
		"distance":               "المسافة",
		"time":                   "الوقت",
		"date":                   "التاريخ",
		"today":                  "اليوم",
		"yesterday":              "أمس",
		"tomorrow":               "غداً",
		"week":                   "الأسبوع",
		"month":                  "الشهر",
		"year":                   "السنة",
		"morning":                "الصباح",
		"afternoon":              "بعد الظهر",
		"evening":                "المساء",
		"night":                  "الليل",
		"prayer_times":           "أوقات الصلاة",
		"fajr":                   "الفجر",
		"dhuhr":                  "الظهر",
		"asr":                    "العصر",
		"maghrib":                "المغرب",
		"isha":                   "العشاء",
		"ramadan":                "رمضان",
		"eid":                    "العيد",
		"hajj":                   "الحج",
		"umrah":                  "العمرة",
		"friday":                 "الجمعة",
		"saturday":               "السبت",
		"sunday":                 "الأحد",
		"monday":                 "الاثنين",
		"tuesday":                "الثلاثاء",
		"wednesday":              "الأربعاء",
		"thursday":               "الخميس",
	}
	
	// Initialize English translations
	translations["en"] = map[string]string{
		"welcome":                "Welcome",
		"dashboard":              "Dashboard",
		"fleet_management":       "Fleet Management",
		"vehicle":                "Vehicle",
		"vehicles":               "Vehicles",
		"status":                 "Status",
		"active":                 "Active",
		"inactive":               "Inactive",
		"maintenance":            "Maintenance",
		"route":                  "Route",
		"driver":                 "Driver",
		"passenger":              "Passenger",
		"location":               "Location",
		"destination":            "Destination",
		"departure":              "Departure",
		"arrival":                "Arrival",
		"schedule":               "Schedule",
		"booking":                "Booking",
		"cancel":                 "Cancel",
		"confirm":                "Confirm",
		"save":                   "Save",
		"edit":                   "Edit",
		"delete":                 "Delete",
		"search":                 "Search",
		"filter":                 "Filter",
		"sort":                   "Sort",
		"settings":               "Settings",
		"profile":                "Profile",
		"logout":                 "Logout",
		"login":                  "Login",
		"username":               "Username",
		"password":               "Password",
		"email":                  "Email",
		"phone":                  "Phone",
		"address":                "Address",
		"city":                   "City",
		"country":                "Country",
		"uae":                    "United Arab Emirates",
		"dubai":                  "Dubai",
		"abu_dhabi":              "Abu Dhabi",
		"sharjah":                "Sharjah",
		"ajman":                  "Ajman",
		"ras_al_khaimah":         "Ras Al Khaimah",
		"fujairah":               "Fujairah",
		"umm_al_quwain":          "Umm Al Quwain",
		"emergency":              "Emergency",
		"police":                 "Police",
		"ambulance":              "Ambulance",
		"fire_department":        "Fire Department",
		"traffic":                "Traffic",
		"weather":                "Weather",
		"temperature":            "Temperature",
		"humidity":               "Humidity",
		"wind":                   "Wind",
		"rain":                   "Rain",
		"sunny":                  "Sunny",
		"cloudy":                 "Cloudy",
		"safety":                 "Safety",
		"security":               "Security",
		"compliance":             "Compliance",
		"regulation":             "Regulation",
		"license":                "License",
		"permit":                 "Permit",
		"inspection":             "Inspection",
		"audit":                  "Audit",
		"report":                 "Report",
		"analytics":              "Analytics",
		"performance":            "Performance",
		"efficiency":             "Efficiency",
		"cost":                   "Cost",
		"revenue":                "Revenue",
		"profit":                 "Profit",
		"fuel":                   "Fuel",
		"energy":                 "Energy",
		"battery":                "Battery",
		"charging":               "Charging",
		"electric":               "Electric",
		"autonomous":             "Autonomous",
		"manual":                 "Manual",
		"automatic":              "Automatic",
		"speed":                  "Speed",
		"distance":               "Distance",
		"time":                   "Time",
		"date":                   "Date",
		"today":                  "Today",
		"yesterday":              "Yesterday",
		"tomorrow":               "Tomorrow",
		"week":                   "Week",
		"month":                  "Month",
		"year":                   "Year",
		"morning":                "Morning",
		"afternoon":              "Afternoon",
		"evening":                "Evening",
		"night":                  "Night",
		"prayer_times":           "Prayer Times",
		"fajr":                   "Fajr",
		"dhuhr":                  "Dhuhr",
		"asr":                    "Asr",
		"maghrib":                "Maghrib",
		"isha":                   "Isha",
		"ramadan":                "Ramadan",
		"eid":                    "Eid",
		"hajj":                   "Hajj",
		"umrah":                  "Umrah",
		"friday":                 "Friday",
		"saturday":               "Saturday",
		"sunday":                 "Sunday",
		"monday":                 "Monday",
		"tuesday":                "Tuesday",
		"wednesday":              "Wednesday",
		"thursday":               "Thursday",
	}
	
	contexts := make(map[string]TranslationContext)
	contexts["fleet_operations"] = TranslationContext{
		Domain:   "fleet_management",
		Context:  "operations",
		Metadata: map[string]interface{}{"formal": true, "technical": true},
	}
	
	return &TranslationEngine{
		translations: translations,
		contexts:     contexts,
	}
}

func initIslamicCalendarService() *IslamicCalendarService {
	calendarData := make(map[string]IslamicDate)
	holidays := make(map[string][]Holiday)
	prayerTimes := make(map[string]PrayerSchedule)
	
	// Initialize UAE holidays for 2024
	holidays["2024"] = []Holiday{
		{
			HolidayID:     "new_year_2024",
			Name:          "New Year's Day",
			NameArabic:    "رأس السنة الميلادية",
			Type:          "national",
			Date:          time.Date(2024, 1, 1, 0, 0, 0, 0, time.UTC),
			Duration:      1,
			Description:   "New Year's Day celebration",
			DescriptionAr: "احتفال رأس السنة الميلادية",
			WorkingDay:    false,
		},
		{
			HolidayID:     "eid_al_fitr_2024",
			Name:          "Eid Al Fitr",
			NameArabic:    "عيد الفطر",
			Type:          "religious",
			Date:          time.Date(2024, 4, 10, 0, 0, 0, 0, time.UTC),
			Duration:      3,
			Description:   "Festival of Breaking the Fast",
			DescriptionAr: "عيد الفطر المبارك",
			WorkingDay:    false,
		},
		{
			HolidayID:     "eid_al_adha_2024",
			Name:          "Eid Al Adha",
			NameArabic:    "عيد الأضحى",
			Type:          "religious",
			Date:          time.Date(2024, 6, 16, 0, 0, 0, 0, time.UTC),
			Duration:      4,
			Description:   "Festival of Sacrifice",
			DescriptionAr: "عيد الأضحى المبارك",
			WorkingDay:    false,
		},
		{
			HolidayID:     "uae_national_day_2024",
			Name:          "UAE National Day",
			NameArabic:    "اليوم الوطني لدولة الإمارات",
			Type:          "national",
			Date:          time.Date(2024, 12, 2, 0, 0, 0, 0, time.UTC),
			Duration:      2,
			Description:   "UAE National Day celebration",
			DescriptionAr: "احتفال اليوم الوطني لدولة الإمارات",
			WorkingDay:    false,
		},
	}
	
	return &IslamicCalendarService{
		calendarData: calendarData,
		holidays:     holidays,
		prayerTimes:  prayerTimes,
	}
}

func initCulturalAdapter() *CulturalAdapter {
	culturalRules := make(map[string]CulturalRule)
	adaptations := make(map[string]CulturalAdaptation)
	
	// Initialize cultural rules
	culturalRules["prayer_time_awareness"] = CulturalRule{
		RuleID:      "prayer_time_awareness",
		Category:    "religious",
		Description: "Adjust operations during prayer times",
		Context:     "scheduling",
		Rules: map[string]interface{}{
			"avoid_scheduling_during_prayers": true,
			"buffer_time_minutes":             15,
			"priority_prayers":                []string{"fajr", "dhuhr", "asr", "maghrib", "isha"},
		},
		Priority: 1,
		Active:   true,
	}
	
	culturalRules["ramadan_adjustments"] = CulturalRule{
		RuleID:      "ramadan_adjustments",
		Category:    "religious",
		Description: "Operational adjustments during Ramadan",
		Context:     "scheduling",
		Rules: map[string]interface{}{
			"reduced_daytime_operations": true,
			"iftar_break_duration":       60,
			"suhoor_considerations":      true,
		},
		Priority: 1,
		Active:   true,
	}
	
	// Initialize adaptations
	adaptations["ui_rtl"] = CulturalAdaptation{
		AdaptationID: "ui_rtl",
		Type:         "ui",
		Target:       "interface",
		Modifications: map[string]interface{}{
			"text_direction": "rtl",
			"layout_mirror": true,
			"font_family":   "Noto Sans Arabic",
		},
		Conditions: []string{"language:ar"},
		Active:     true,
	}
	
	return &CulturalAdapter{
		culturalRules: culturalRules,
		adaptations:   adaptations,
	}
}

func initRTLProcessor() *RTLProcessor {
	rtlRules := make(map[string]RTLRule)
	processors := make(map[string]TextProcessor)
	
	// Initialize RTL rules
	rtlRules["arabic_rtl"] = RTLRule{
		RuleID:    "arabic_rtl",
		Language:  "ar",
		Script:    "arabic",
		Direction: "rtl",
		Alignment: "right",
		Components: []string{"text", "input", "button", "menu"},
		Active:    true,
	}
	
	// Initialize processors
	processors["arabic_bidi"] = TextProcessor{
		ProcessorID: "arabic_bidi",
		Type:        "bidi",
		Language:    "ar",
		Rules: map[string]interface{}{
			"unicode_bidi": "bidi-override",
			"direction":    "rtl",
		},
		Priority: 1,
	}
	
	return &RTLProcessor{
		rtlRules:   rtlRules,
		processors: processors,
	}
}

// Translation Methods
func (s *ArabicLocalizationService) translateText(w http.ResponseWriter, r *http.Request) {
	var request TranslationRequest
	if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	// Get translations for the requested language
	langTranslations, exists := s.translationEngine.translations[request.Language]
	if !exists {
		// Fallback to English
		langTranslations = s.translationEngine.translations["en"]
		request.Language = "en"
	}

	translations := make(map[string]string)
	for _, key := range request.Keys {
		if translation, exists := langTranslations[key]; exists {
			translations[key] = translation
		} else {
			// Fallback to key itself
			translations[key] = key
		}
	}

	response := TranslationResponse{
		Translations: translations,
		Language:     request.Language,
		Context:      request.Context,
		Metadata: map[string]interface{}{
			"total_keys":     len(request.Keys),
			"translated":     len(translations),
			"fallback_used":  request.Language == "en",
		},
		Timestamp: time.Now(),
	}

	log.Printf("✅ Translated %d keys to %s", len(translations), request.Language)
	s.sendJSON(w, http.StatusOK, response)
}

func (s *ArabicLocalizationService) getSupportedLanguages(w http.ResponseWriter, r *http.Request) {
	languages := []map[string]interface{}{
		{
			"code":        "ar",
			"name":        "Arabic",
			"native_name": "العربية",
			"direction":   "rtl",
			"script":      "arabic",
			"region":      "UAE",
		},
		{
			"code":        "en",
			"name":        "English",
			"native_name": "English",
			"direction":   "ltr",
			"script":      "latin",
			"region":      "UAE",
		},
	}

	log.Printf("✅ Retrieved %d supported languages", len(languages))
	s.sendJSON(w, http.StatusOK, languages)
}

func (s *ArabicLocalizationService) getTranslationKeys(w http.ResponseWriter, r *http.Request) {
	language := r.URL.Query().Get("language")
	if language == "" {
		language = "en"
	}

	langTranslations, exists := s.translationEngine.translations[language]
	if !exists {
		s.handleError(w, "Language not supported", nil, http.StatusBadRequest)
		return
	}

	keys := make([]string, 0, len(langTranslations))
	for key := range langTranslations {
		keys = append(keys, key)
	}

	response := map[string]interface{}{
		"language": language,
		"keys":     keys,
		"count":    len(keys),
	}

	log.Printf("✅ Retrieved %d translation keys for %s", len(keys), language)
	s.sendJSON(w, http.StatusOK, response)
}

func (s *ArabicLocalizationService) getTranslationContexts(w http.ResponseWriter, r *http.Request) {
	contexts := make([]TranslationContext, 0, len(s.translationEngine.contexts))
	for _, context := range s.translationEngine.contexts {
		contexts = append(contexts, context)
	}

	log.Printf("✅ Retrieved %d translation contexts", len(contexts))
	s.sendJSON(w, http.StatusOK, contexts)
}

// Islamic Calendar Methods
func (s *ArabicLocalizationService) getIslamicDate(w http.ResponseWriter, r *http.Request) {
	dateStr := r.URL.Query().Get("date")
	var targetDate time.Time
	var err error

	if dateStr == "" {
		targetDate = time.Now()
	} else {
		targetDate, err = time.Parse("2006-01-02", dateStr)
		if err != nil {
			s.handleError(w, "Invalid date format", err, http.StatusBadRequest)
			return
		}
	}

	// Convert to Islamic date (simplified calculation)
	islamicDate := s.convertToIslamicDate(targetDate)

	log.Printf("✅ Converted date %s to Islamic date", targetDate.Format("2006-01-02"))
	s.sendJSON(w, http.StatusOK, islamicDate)
}

func (s *ArabicLocalizationService) convertToIslamicDate(gregorianDate time.Time) IslamicDate {
	// Simplified Islamic date conversion (in production, use proper astronomical calculations)
	// This is a mock implementation
	epochDiff := gregorianDate.Sub(time.Date(622, 7, 16, 0, 0, 0, 0, time.UTC))
	islamicDays := int(epochDiff.Hours() / 24)
	
	// Approximate conversion (Islamic year is about 354 days)
	islamicYear := 1 + islamicDays/354
	remainingDays := islamicDays % 354
	islamicMonth := 1 + remainingDays/29
	islamicDay := 1 + remainingDays%29

	monthNames := []string{
		"Muharram", "Safar", "Rabi' al-awwal", "Rabi' al-thani",
		"Jumada al-awwal", "Jumada al-thani", "Rajab", "Sha'ban",
		"Ramadan", "Shawwal", "Dhu al-Qi'dah", "Dhu al-Hijjah",
	}
	
	monthNamesAr := []string{
		"محرم", "صفر", "ربيع الأول", "ربيع الثاني",
		"جمادى الأولى", "جمادى الثانية", "رجب", "شعبان",
		"رمضان", "شوال", "ذو القعدة", "ذو الحجة",
	}

	weekDays := []string{"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"}
	weekDaysAr := []string{"الأحد", "الاثنين", "الثلاثاء", "الأربعاء", "الخميس", "الجمعة", "السبت"}

	if islamicMonth > 12 {
		islamicMonth = 12
	}
	if islamicDay > 29 {
		islamicDay = 29
	}

	return IslamicDate{
		HijriYear:     islamicYear,
		HijriMonth:    islamicMonth,
		HijriDay:      islamicDay,
		MonthName:     monthNames[islamicMonth-1],
		MonthNameAr:   monthNamesAr[islamicMonth-1],
		WeekDay:       weekDays[gregorianDate.Weekday()],
		WeekDayAr:     weekDaysAr[gregorianDate.Weekday()],
		GregorianDate: gregorianDate,
	}
}

func (s *ArabicLocalizationService) getHolidays(w http.ResponseWriter, r *http.Request) {
	year := r.URL.Query().Get("year")
	if year == "" {
		year = fmt.Sprintf("%d", time.Now().Year())
	}

	holidays, exists := s.calendarService.holidays[year]
	if !exists {
		holidays = []Holiday{} // Return empty array if no holidays for the year
	}

	response := map[string]interface{}{
		"year":     year,
		"holidays": holidays,
		"count":    len(holidays),
	}

	log.Printf("✅ Retrieved %d holidays for year %s", len(holidays), year)
	s.sendJSON(w, http.StatusOK, response)
}

func (s *ArabicLocalizationService) getPrayerTimes(w http.ResponseWriter, r *http.Request) {
	dateStr := r.URL.Query().Get("date")
	location := r.URL.Query().Get("location")
	
	if location == "" {
		location = "Dubai" // Default to Dubai
	}

	var targetDate time.Time
	var err error

	if dateStr == "" {
		targetDate = time.Now()
	} else {
		targetDate, err = time.Parse("2006-01-02", dateStr)
		if err != nil {
			s.handleError(w, "Invalid date format", err, http.StatusBadRequest)
			return
		}
	}

	// Generate prayer times (simplified calculation)
	prayerSchedule := s.generatePrayerTimes(targetDate, location)

	log.Printf("✅ Generated prayer times for %s on %s", location, targetDate.Format("2006-01-02"))
	s.sendJSON(w, http.StatusOK, prayerSchedule)
}

func (s *ArabicLocalizationService) generatePrayerTimes(date time.Time, location string) PrayerSchedule {
	// Simplified prayer time calculation (in production, use proper astronomical calculations)
	// This is a mock implementation for Dubai coordinates
	
	coordinates := []float64{25.2048, 55.2708} // Dubai coordinates
	
	prayers := []PrayerTime{
		{
			Name:     "Fajr",
			NameAr:   "الفجر",
			Time:     time.Date(date.Year(), date.Month(), date.Day(), 5, 30, 0, 0, date.Location()),
			Type:     "fard",
			Duration: 20,
		},
		{
			Name:     "Dhuhr",
			NameAr:   "الظهر",
			Time:     time.Date(date.Year(), date.Month(), date.Day(), 12, 15, 0, 0, date.Location()),
			Type:     "fard",
			Duration: 30,
		},
		{
			Name:     "Asr",
			NameAr:   "العصر",
			Time:     time.Date(date.Year(), date.Month(), date.Day(), 15, 45, 0, 0, date.Location()),
			Type:     "fard",
			Duration: 25,
		},
		{
			Name:     "Maghrib",
			NameAr:   "المغرب",
			Time:     time.Date(date.Year(), date.Month(), date.Day(), 18, 30, 0, 0, date.Location()),
			Type:     "fard",
			Duration: 20,
		},
		{
			Name:     "Isha",
			NameAr:   "العشاء",
			Time:     time.Date(date.Year(), date.Month(), date.Day(), 20, 0, 0, 0, date.Location()),
			Type:     "fard",
			Duration: 30,
		},
	}

	return PrayerSchedule{
		Date:        date,
		Location:    location,
		Coordinates: coordinates,
		Prayers:     prayers,
		Sunrise:     time.Date(date.Year(), date.Month(), date.Day(), 6, 0, 0, 0, date.Location()),
		Sunset:      time.Date(date.Year(), date.Month(), date.Day(), 18, 0, 0, 0, date.Location()),
		QiblaDirection: 258.5, // Qibla direction from Dubai to Mecca
	}
}

func (s *ArabicLocalizationService) convertDate(w http.ResponseWriter, r *http.Request) {
	var request struct {
		Date     string `json:"date"`
		FromType string `json:"from_type"` // gregorian, islamic
		ToType   string `json:"to_type"`   // gregorian, islamic
	}

	if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	// Mock date conversion
	response := map[string]interface{}{
		"original_date": request.Date,
		"from_type":     request.FromType,
		"to_type":       request.ToType,
		"converted_date": "1445-06-15", // Mock Islamic date
		"timestamp":     time.Now(),
	}

	log.Printf("✅ Converted date from %s to %s", request.FromType, request.ToType)
	s.sendJSON(w, http.StatusOK, response)
}

// Cultural Adaptation Methods
func (s *ArabicLocalizationService) applyCulturalAdaptations(w http.ResponseWriter, r *http.Request) {
	var request struct {
		Context     CulturalContext            `json:"context"`
		Content     map[string]interface{}     `json:"content"`
		Adaptations []string                   `json:"adaptations"`
	}

	if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	// Apply cultural adaptations
	adaptedContent := s.applyCulturalRules(request.Content, request.Context)

	response := map[string]interface{}{
		"original_content": request.Content,
		"adapted_content":  adaptedContent,
		"context":          request.Context,
		"adaptations_applied": len(request.Adaptations),
		"timestamp":        time.Now(),
	}

	log.Printf("✅ Applied %d cultural adaptations", len(request.Adaptations))
	s.sendJSON(w, http.StatusOK, response)
}

func (s *ArabicLocalizationService) applyCulturalRules(content map[string]interface{}, context CulturalContext) map[string]interface{} {
	adaptedContent := make(map[string]interface{})
	
	// Copy original content
	for key, value := range content {
		adaptedContent[key] = value
	}

	// Apply language-specific adaptations
	if context.Language == "ar" {
		adaptedContent["text_direction"] = "rtl"
		adaptedContent["font_family"] = "Noto Sans Arabic"
		adaptedContent["alignment"] = "right"
	}

	// Apply religious considerations
	if context.Religion == "islam" {
		adaptedContent["prayer_time_awareness"] = true
		adaptedContent["halal_considerations"] = true
	}

	// Apply regional adaptations
	if context.Region == "UAE" {
		adaptedContent["currency"] = "AED"
		adaptedContent["timezone"] = "Asia/Dubai"
		adaptedContent["weekend"] = []string{"Friday", "Saturday"}
	}

	return adaptedContent
}

func (s *ArabicLocalizationService) getCulturalRules(w http.ResponseWriter, r *http.Request) {
	rules := make([]CulturalRule, 0, len(s.culturalAdapter.culturalRules))
	for _, rule := range s.culturalAdapter.culturalRules {
		rules = append(rules, rule)
	}

	log.Printf("✅ Retrieved %d cultural rules", len(rules))
	s.sendJSON(w, http.StatusOK, rules)
}

func (s *ArabicLocalizationService) getCulturalContext(w http.ResponseWriter, r *http.Request) {
	// Mock cultural context based on request headers or parameters
	context := CulturalContext{
		Region:    "UAE",
		Language:  "ar",
		Religion:  "islam",
		Preferences: []string{"rtl_ui", "prayer_times", "islamic_calendar"},
		Timestamp: time.Now(),
	}

	log.Printf("✅ Generated cultural context for region: %s", context.Region)
	s.sendJSON(w, http.StatusOK, context)
}

// RTL Processing Methods
func (s *ArabicLocalizationService) processRTL(w http.ResponseWriter, r *http.Request) {
	var request RTLProcessingRequest
	if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	// Process RTL text
	response := s.processRTLText(request)

	log.Printf("✅ Processed RTL text for language: %s", request.Language)
	s.sendJSON(w, http.StatusOK, response)
}

func (s *ArabicLocalizationService) processRTLText(request RTLProcessingRequest) RTLProcessingResponse {
	// Mock RTL processing
	processedText := request.Text
	direction := "ltr"
	alignment := "left"

	if request.Language == "ar" {
		direction = "rtl"
		alignment = "right"
		// In production, apply proper bidirectional text processing
	}

	styling := map[string]interface{}{
		"direction":    direction,
		"text-align":   alignment,
		"font-family":  "Noto Sans Arabic",
		"unicode-bidi": "bidi-override",
	}

	return RTLProcessingResponse{
		ProcessedText: processedText,
		Direction:     direction,
		Alignment:     alignment,
		Styling:       styling,
		Metadata: map[string]interface{}{
			"original_length": len(request.Text),
			"processed_length": len(processedText),
			"language": request.Language,
		},
		Timestamp: time.Now(),
	}
}

func (s *ArabicLocalizationService) getRTLRules(w http.ResponseWriter, r *http.Request) {
	rules := make([]RTLRule, 0, len(s.rtlProcessor.rtlRules))
	for _, rule := range s.rtlProcessor.rtlRules {
		rules = append(rules, rule)
	}

	log.Printf("✅ Retrieved %d RTL rules", len(rules))
	s.sendJSON(w, http.StatusOK, rules)
}

func (s *ArabicLocalizationService) validateRTL(w http.ResponseWriter, r *http.Request) {
	var request struct {
		Text     string `json:"text"`
		Language string `json:"language"`
		Rules    []string `json:"rules"`
	}

	if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	// Mock RTL validation
	validation := map[string]interface{}{
		"valid":        true,
		"issues":       []string{},
		"suggestions":  []string{"Consider using proper Arabic font", "Ensure proper text direction"},
		"score":        0.95,
		"language":     request.Language,
		"rules_checked": len(request.Rules),
		"timestamp":    time.Now(),
	}

	log.Printf("✅ Validated RTL text for language: %s", request.Language)
	s.sendJSON(w, http.StatusOK, validation)
}

// Utility Methods
func (s *ArabicLocalizationService) healthCheck(w http.ResponseWriter, r *http.Request) {
	health := map[string]interface{}{
		"status":    "healthy",
		"timestamp": time.Now(),
		"components": map[string]interface{}{
			"translation_engine": map[string]interface{}{
				"languages_supported": len(s.translationEngine.translations),
				"contexts_loaded":     len(s.translationEngine.contexts),
			},
			"calendar_service": map[string]interface{}{
				"holidays_loaded": len(s.calendarService.holidays),
			},
			"cultural_adapter": map[string]interface{}{
				"rules_loaded":       len(s.culturalAdapter.culturalRules),
				"adaptations_loaded": len(s.culturalAdapter.adaptations),
			},
			"rtl_processor": map[string]interface{}{
				"rules_loaded":      len(s.rtlProcessor.rtlRules),
				"processors_loaded": len(s.rtlProcessor.processors),
			},
		},
		"version": "1.0.0",
	}

	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	json.NewEncoder(w).Encode(health)
}

func (s *ArabicLocalizationService) handleError(w http.ResponseWriter, message string, err error, statusCode int) {
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

func (s *ArabicLocalizationService) sendJSON(w http.ResponseWriter, statusCode int, data interface{}) {
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
