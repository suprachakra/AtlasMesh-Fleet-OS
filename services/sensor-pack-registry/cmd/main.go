package main

import (
"context"
"log"
"net/http"
"os"
"os/signal"
"syscall"
"time"

"github.com/gin-gonic/gin"
"github.com/spf13/viper"
)

func main() {
log.Println("Starting Sensor Pack Registry service")

if err := loadConfig(); err != nil {
log.Fatalf("failed to load configuration: %v", err)
}

r := gin.Default()

r.GET("/health", func(c *gin.Context) {
c.JSON(http.StatusOK, gin.H{"status": "healthy"})
})

srv := &http.Server{
Addr:    ":8082",
Handler: r,
}

go func() {
if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
log.Fatalf("server error: %v", err)
}
}()

quit := make(chan os.Signal, 1)
signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
<-quit

ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
defer cancel()
if err := srv.Shutdown(ctx); err != nil {
log.Fatalf("server shutdown failed: %v", err)
}
}

func loadConfig() error {
viper.SetDefault("PACK_SCHEMA_PATH", "configs/sensor-packs/schema.json")
return nil
}
