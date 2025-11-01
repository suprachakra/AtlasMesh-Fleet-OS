#!/usr/bin/env python3
"""
AtlasMesh Fleet OS - Fleet Telemetry ETL Job
Apache Spark job for processing vehicle telemetry data
"""

import os
import sys
from datetime import datetime, timedelta
from typing import Dict, List, Optional

from pyspark.sql import SparkSession, DataFrame
from pyspark.sql.functions import (
    col, when, isnan, isnull, count, mean, stddev, min as spark_min, max as spark_max,
    window, to_timestamp, from_json, explode, array_contains, size, split,
    regexp_replace, trim, upper, lower, coalesce, lit, current_timestamp,
    year, month, dayofmonth, hour, minute, date_format, unix_timestamp,
    lag, lead, row_number, rank, dense_rank, ntile, percent_rank,
    sum as spark_sum, avg, collect_list, collect_set, first, last,
    monotonically_increasing_id, broadcast, expr
)
from pyspark.sql.types import (
    StructType, StructField, StringType, IntegerType, FloatType, 
    DoubleType, BooleanType, TimestampType, ArrayType, MapType
)
from pyspark.sql.window import Window
import pyspark.sql.functions as F

class FleetTelemetryETL:
    """
    ETL processor for fleet telemetry data
    Handles data extraction, transformation, and loading for analytics
    """
    
    def __init__(self, spark_session: SparkSession):
        self.spark = spark_session
        self.config = self._load_config()
        
    def _load_config(self) -> Dict:
        """Load ETL configuration from environment variables"""
        return {
            'postgres': {
                'host': os.getenv('POSTGRES_HOST', 'postgres'),
                'port': os.getenv('POSTGRES_PORT', '5432'),
                'database': os.getenv('POSTGRES_DB', 'atlasmesh_fleet_os'),
                'user': os.getenv('POSTGRES_USER', 'postgres'),
                'password': os.getenv('POSTGRES_PASSWORD', ''),
                'driver': 'org.postgresql.Driver'
            },
            'clickhouse': {
                'host': os.getenv('CLICKHOUSE_HOST', 'clickhouse-01'),
                'port': os.getenv('CLICKHOUSE_PORT', '8123'),
                'database': os.getenv('CLICKHOUSE_DB', 'atlasmesh_analytics'),
                'user': os.getenv('CLICKHOUSE_USER', 'analytics_user'),
                'password': os.getenv('CLICKHOUSE_PASSWORD', ''),
            },
            'kafka': {
                'brokers': os.getenv('KAFKA_BROKERS', 'kafka:9092'),
                'topics': {
                    'telemetry': 'vehicle-telemetry',
                    'trips': 'trip-events',
                    'alerts': 'vehicle-alerts'
                }
            },
            'minio': {
                'endpoint': os.getenv('MINIO_ENDPOINT', 'minio:9000'),
                'access_key': os.getenv('MINIO_ACCESS_KEY', 'minioadmin'),
                'secret_key': os.getenv('MINIO_SECRET_KEY', 'minioadmin123'),
                'bucket': os.getenv('MINIO_BUCKET', 'atlasmesh-datalake')
            },
            'processing': {
                'batch_size': int(os.getenv('BATCH_SIZE', '10000')),
                'window_duration': os.getenv('WINDOW_DURATION', '5 minutes'),
                'watermark_delay': os.getenv('WATERMARK_DELAY', '1 minute'),
                'checkpoint_location': os.getenv('CHECKPOINT_LOCATION', '/tmp/checkpoints')
            }
        }
    
    def extract_telemetry_data(self, start_time: datetime, end_time: datetime) -> DataFrame:
        """Extract telemetry data from PostgreSQL"""
        print(f"📥 Extracting telemetry data from {start_time} to {end_time}")
        
        postgres_url = f"jdbc:postgresql://{self.config['postgres']['host']}:{self.config['postgres']['port']}/{self.config['postgres']['database']}"
        
        # Define the query to extract telemetry data
        query = f"""
        (SELECT 
            vt.id,
            vt.vehicle_id,
            vt.timestamp,
            vt.location_lat,
            vt.location_lng,
            vt.speed_kmh,
            vt.fuel_level_percent,
            vt.battery_level_percent,
            vt.engine_temperature,
            vt.odometer_km,
            vt.status,
            vt.sensor_data,
            v.fleet_id,
            v.vehicle_type,
            v.license_plate,
            f.name as fleet_name
        FROM vehicle_telemetry vt
        JOIN vehicles v ON vt.vehicle_id = v.vehicle_id
        JOIN fleets f ON v.fleet_id = f.fleet_id
        WHERE vt.timestamp >= '{start_time.strftime('%Y-%m-%d %H:%M:%S')}'
        AND vt.timestamp < '{end_time.strftime('%Y-%m-%d %H:%M:%S')}'
        ) as telemetry_data
        """
        
        df = self.spark.read \
            .format("jdbc") \
            .option("url", postgres_url) \
            .option("dbtable", query) \
            .option("user", self.config['postgres']['user']) \
            .option("password", self.config['postgres']['password']) \
            .option("driver", self.config['postgres']['driver']) \
            .option("fetchsize", str(self.config['processing']['batch_size'])) \
            .load()
        
        print(f"✅ Extracted {df.count()} telemetry records")
        return df
    
    def extract_trip_data(self, start_time: datetime, end_time: datetime) -> DataFrame:
        """Extract trip data from PostgreSQL"""
        print(f"📥 Extracting trip data from {start_time} to {end_time}")
        
        postgres_url = f"jdbc:postgresql://{self.config['postgres']['host']}:{self.config['postgres']['port']}/{self.config['postgres']['database']}"
        
        query = f"""
        (SELECT 
            t.trip_id,
            t.vehicle_id,
            t.driver_id,
            t.start_time,
            t.end_time,
            t.pickup_lat,
            t.pickup_lng,
            t.dropoff_lat,
            t.dropoff_lng,
            t.distance_km,
            t.duration_minutes,
            t.fare,
            t.status,
            t.rating,
            v.fleet_id,
            v.vehicle_type,
            d.first_name || ' ' || d.last_name as driver_name
        FROM trips t
        JOIN vehicles v ON t.vehicle_id = v.vehicle_id
        LEFT JOIN drivers d ON t.driver_id = d.driver_id
        WHERE t.start_time >= '{start_time.strftime('%Y-%m-%d %H:%M:%S')}'
        AND t.start_time < '{end_time.strftime('%Y-%m-%d %H:%M:%S')}'
        ) as trip_data
        """
        
        df = self.spark.read \
            .format("jdbc") \
            .option("url", postgres_url) \
            .option("dbtable", query) \
            .option("user", self.config['postgres']['user']) \
            .option("password", self.config['postgres']['password']) \
            .option("driver", self.config['postgres']['driver']) \
            .load()
        
        print(f"✅ Extracted {df.count()} trip records")
        return df
    
    def transform_telemetry_data(self, df: DataFrame) -> DataFrame:
        """Transform telemetry data for analytics"""
        print("🔄 Transforming telemetry data...")
        
        # Data cleaning and validation
        df_clean = df.filter(
            (col("location_lat").between(-90, 90)) &
            (col("location_lng").between(-180, 180)) &
            (col("speed_kmh") >= 0) &
            (col("speed_kmh") <= 300) &
            (col("fuel_level_percent").between(0, 100)) &
            (col("battery_level_percent").between(0, 100))
        )
        
        # Add derived columns
        df_transformed = df_clean.withColumn(
            "date", date_format(col("timestamp"), "yyyy-MM-dd")
        ).withColumn(
            "hour", hour(col("timestamp"))
        ).withColumn(
            "day_of_week", date_format(col("timestamp"), "EEEE")
        ).withColumn(
            "is_weekend", 
            when(date_format(col("timestamp"), "EEEE").isin(["Saturday", "Sunday"]), True).otherwise(False)
        ).withColumn(
            "speed_category",
            when(col("speed_kmh") == 0, "Stationary")
            .when(col("speed_kmh") <= 30, "City")
            .when(col("speed_kmh") <= 80, "Highway")
            .otherwise("High Speed")
        ).withColumn(
            "fuel_status",
            when(col("fuel_level_percent") < 10, "Critical")
            .when(col("fuel_level_percent") < 25, "Low")
            .when(col("fuel_level_percent") < 75, "Normal")
            .otherwise("Full")
        ).withColumn(
            "battery_status",
            when(col("battery_level_percent") < 10, "Critical")
            .when(col("battery_level_percent") < 25, "Low")
            .when(col("battery_level_percent") < 75, "Normal")
            .otherwise("Full")
        )
        
        # Calculate vehicle health score
        df_transformed = df_transformed.withColumn(
            "health_score",
            (
                (col("fuel_level_percent") * 0.3) +
                (col("battery_level_percent") * 0.3) +
                (when(col("engine_temperature") <= 90, 40).otherwise(0)) +
                (when(col("status") == "active", 0).otherwise(-10))
            ).cast("integer")
        )
        
        # Add geospatial features (Dubai/UAE specific)
        df_transformed = df_transformed.withColumn(
            "location_zone",
            when(
                (col("location_lat").between(25.0, 25.5)) & 
                (col("location_lng").between(55.0, 55.5)), "Dubai_Central"
            ).when(
                (col("location_lat").between(24.8, 25.0)) & 
                (col("location_lng").between(55.0, 55.3)), "Dubai_South"
            ).when(
                (col("location_lat").between(25.5, 26.0)) & 
                (col("location_lng").between(55.5, 56.0)), "Sharjah"
            ).otherwise("Other_Emirates")
        )
        
        print(f"✅ Transformed telemetry data: {df_transformed.count()} records")
        return df_transformed
    
    def transform_trip_data(self, df: DataFrame) -> DataFrame:
        """Transform trip data for analytics"""
        print("🔄 Transforming trip data...")
        
        # Data cleaning
        df_clean = df.filter(
            (col("distance_km") > 0) &
            (col("duration_minutes") > 0) &
            (col("pickup_lat").between(-90, 90)) &
            (col("pickup_lng").between(-180, 180)) &
            (col("dropoff_lat").between(-90, 90)) &
            (col("dropoff_lng").between(-180, 180))
        )
        
        # Add derived columns
        df_transformed = df_clean.withColumn(
            "date", date_format(col("start_time"), "yyyy-MM-dd")
        ).withColumn(
            "start_hour", hour(col("start_time"))
        ).withColumn(
            "day_of_week", date_format(col("start_time"), "EEEE")
        ).withColumn(
            "is_weekend", 
            when(date_format(col("start_time"), "EEEE").isin(["Saturday", "Sunday"]), True).otherwise(False)
        ).withColumn(
            "avg_speed_kmh",
            (col("distance_km") / (col("duration_minutes") / 60.0))
        ).withColumn(
            "revenue_per_km",
            when(col("distance_km") > 0, col("fare") / col("distance_km")).otherwise(0)
        ).withColumn(
            "trip_category",
            when(col("distance_km") <= 5, "Short")
            .when(col("distance_km") <= 20, "Medium")
            .otherwise("Long")
        ).withColumn(
            "time_category",
            when(col("start_hour").between(6, 9), "Morning_Rush")
            .when(col("start_hour").between(17, 20), "Evening_Rush")
            .when(col("start_hour").between(10, 16), "Midday")
            .when(col("start_hour").between(21, 23), "Evening")
            .otherwise("Night")
        )
        
        # Calculate trip efficiency metrics
        df_transformed = df_transformed.withColumn(
            "efficiency_score",
            when(col("avg_speed_kmh") > 0,
                (col("distance_km") / col("duration_minutes")) * 60 * 
                when(col("rating").isNotNull(), col("rating") / 5.0).otherwise(0.8)
            ).otherwise(0)
        )
        
        print(f"✅ Transformed trip data: {df_transformed.count()} records")
        return df_transformed
    
    def aggregate_fleet_metrics(self, telemetry_df: DataFrame, trip_df: DataFrame) -> DataFrame:
        """Aggregate fleet-level metrics"""
        print("📊 Aggregating fleet metrics...")
        
        # Telemetry aggregations by fleet and hour
        telemetry_agg = telemetry_df.groupBy(
            "fleet_id", "fleet_name", "date", "hour"
        ).agg(
            F.count("*").alias("telemetry_records"),
            F.countDistinct("vehicle_id").alias("active_vehicles"),
            F.avg("speed_kmh").alias("avg_speed"),
            F.avg("fuel_level_percent").alias("avg_fuel_level"),
            F.avg("battery_level_percent").alias("avg_battery_level"),
            F.avg("health_score").alias("avg_health_score"),
            F.sum(when(col("speed_kmh") == 0, 1).otherwise(0)).alias("idle_records"),
            F.sum(when(col("fuel_status") == "Critical", 1).otherwise(0)).alias("critical_fuel_vehicles"),
            F.sum(when(col("battery_status") == "Critical", 1).otherwise(0)).alias("critical_battery_vehicles")
        )
        
        # Trip aggregations by fleet and date
        trip_agg = trip_df.groupBy(
            "fleet_id", "date"
        ).agg(
            F.count("*").alias("total_trips"),
            F.sum(when(col("status") == "completed", 1).otherwise(0)).alias("completed_trips"),
            F.sum(when(col("status") == "cancelled", 1).otherwise(0)).alias("cancelled_trips"),
            F.sum("distance_km").alias("total_distance"),
            F.sum("duration_minutes").alias("total_duration"),
            F.sum("fare").alias("total_revenue"),
            F.avg("rating").alias("avg_rating"),
            F.avg("avg_speed_kmh").alias("avg_trip_speed"),
            F.avg("efficiency_score").alias("avg_efficiency")
        )
        
        # Join telemetry and trip aggregations
        fleet_metrics = telemetry_agg.join(
            trip_agg, 
            ["fleet_id", "date"], 
            "full_outer"
        ).fillna(0)
        
        # Calculate derived metrics
        fleet_metrics = fleet_metrics.withColumn(
            "utilization_rate",
            when(col("active_vehicles") > 0, 
                 (col("telemetry_records") - col("idle_records")) / col("telemetry_records") * 100
            ).otherwise(0)
        ).withColumn(
            "completion_rate",
            when(col("total_trips") > 0,
                 col("completed_trips") / col("total_trips") * 100
            ).otherwise(0)
        ).withColumn(
            "revenue_per_km",
            when(col("total_distance") > 0,
                 col("total_revenue") / col("total_distance")
            ).otherwise(0)
        ).withColumn(
            "avg_trip_duration",
            when(col("completed_trips") > 0,
                 col("total_duration") / col("completed_trips")
            ).otherwise(0)
        ).withColumn(
            "processed_at", current_timestamp()
        )
        
        print(f"✅ Aggregated fleet metrics: {fleet_metrics.count()} records")
        return fleet_metrics
    
    def load_to_clickhouse(self, df: DataFrame, table_name: str, mode: str = "append"):
        """Load data to ClickHouse"""
        print(f"📤 Loading data to ClickHouse table: {table_name}")
        
        clickhouse_url = f"jdbc:clickhouse://{self.config['clickhouse']['host']}:{self.config['clickhouse']['port']}/{self.config['clickhouse']['database']}"
        
        df.write \
            .format("jdbc") \
            .option("url", clickhouse_url) \
            .option("dbtable", table_name) \
            .option("user", self.config['clickhouse']['user']) \
            .option("password", self.config['clickhouse']['password']) \
            .option("driver", "com.clickhouse.jdbc.ClickHouseDriver") \
            .option("batchsize", str(self.config['processing']['batch_size'])) \
            .mode(mode) \
            .save()
        
        print(f"✅ Loaded {df.count()} records to {table_name}")
    
    def save_to_data_lake(self, df: DataFrame, path: str, format: str = "parquet"):
        """Save data to MinIO data lake"""
        print(f"💾 Saving data to data lake: {path}")
        
        # Configure Spark for MinIO
        self.spark.conf.set("spark.hadoop.fs.s3a.endpoint", f"http://{self.config['minio']['endpoint']}")
        self.spark.conf.set("spark.hadoop.fs.s3a.access.key", self.config['minio']['access_key'])
        self.spark.conf.set("spark.hadoop.fs.s3a.secret.key", self.config['minio']['secret_key'])
        self.spark.conf.set("spark.hadoop.fs.s3a.path.style.access", "true")
        self.spark.conf.set("spark.hadoop.fs.s3a.impl", "org.apache.hadoop.fs.s3a.S3AFileSystem")
        
        s3_path = f"s3a://{self.config['minio']['bucket']}/{path}"
        
        df.write \
            .format(format) \
            .mode("append") \
            .partitionBy("date") \
            .save(s3_path)
        
        print(f"✅ Saved data to {s3_path}")
    
    def run_etl_pipeline(self, start_time: datetime, end_time: datetime):
        """Run the complete ETL pipeline"""
        print(f"🚀 Starting ETL pipeline for period: {start_time} to {end_time}")
        
        try:
            # Extract data
            telemetry_df = self.extract_telemetry_data(start_time, end_time)
            trip_df = self.extract_trip_data(start_time, end_time)
            
            # Transform data
            telemetry_transformed = self.transform_telemetry_data(telemetry_df)
            trip_transformed = self.transform_trip_data(trip_df)
            
            # Aggregate metrics
            fleet_metrics = self.aggregate_fleet_metrics(telemetry_transformed, trip_transformed)
            
            # Load to ClickHouse
            self.load_to_clickhouse(telemetry_transformed, "vehicle_telemetry_processed")
            self.load_to_clickhouse(trip_transformed, "trip_data_processed")
            self.load_to_clickhouse(fleet_metrics, "fleet_performance_hourly")
            
            # Save to data lake
            date_partition = start_time.strftime("%Y/%m/%d")
            self.save_to_data_lake(telemetry_transformed, f"telemetry/{date_partition}")
            self.save_to_data_lake(trip_transformed, f"trips/{date_partition}")
            self.save_to_data_lake(fleet_metrics, f"fleet_metrics/{date_partition}")
            
            print("✅ ETL pipeline completed successfully")
            
        except Exception as e:
            print(f"❌ ETL pipeline failed: {str(e)}")
            raise e

def main():
    """Main entry point for the ETL job"""
    
    # Initialize Spark session
    spark = SparkSession.builder \
        .appName("AtlasMesh Fleet Telemetry ETL") \
        .config("spark.sql.adaptive.enabled", "true") \
        .config("spark.sql.adaptive.coalescePartitions.enabled", "true") \
        .config("spark.sql.adaptive.skewJoin.enabled", "true") \
        .config("spark.serializer", "org.apache.spark.serializer.KryoSerializer") \
        .config("spark.sql.execution.arrow.pyspark.enabled", "true") \
        .getOrCreate()
    
    # Set log level
    spark.sparkContext.setLogLevel("WARN")
    
    try:
        # Initialize ETL processor
        etl = FleetTelemetryETL(spark)
        
        # Get time range from command line arguments or use default (last hour)
        if len(sys.argv) >= 3:
            start_time = datetime.strptime(sys.argv[1], "%Y-%m-%d %H:%M:%S")
            end_time = datetime.strptime(sys.argv[2], "%Y-%m-%d %H:%M:%S")
        else:
            end_time = datetime.now().replace(minute=0, second=0, microsecond=0)
            start_time = end_time - timedelta(hours=1)
        
        # Run ETL pipeline
        etl.run_etl_pipeline(start_time, end_time)
        
    except Exception as e:
        print(f"❌ Job failed: {str(e)}")
        sys.exit(1)
    finally:
        spark.stop()

if __name__ == "__main__":
    main()
