#!/bin/bash
set -e

# Create multiple databases for different services
psql -v ON_ERROR_STOP=1 --username "$POSTGRES_USER" --dbname "$POSTGRES_DB" <<-EOSQL
    -- Policy Engine Database
    CREATE DATABASE atlasmesh_policy;
    GRANT ALL PRIVILEGES ON DATABASE atlasmesh_policy TO $POSTGRES_USER;

    -- Trip Service Database
    CREATE DATABASE atlasmesh_trips;
    GRANT ALL PRIVILEGES ON DATABASE atlasmesh_trips TO $POSTGRES_USER;

    -- Dispatch Service Database
    CREATE DATABASE atlasmesh_dispatch;
    GRANT ALL PRIVILEGES ON DATABASE atlasmesh_dispatch TO $POSTGRES_USER;

    -- Routing Service Database
    CREATE DATABASE atlasmesh_routing;
    GRANT ALL PRIVILEGES ON DATABASE atlasmesh_routing TO $POSTGRES_USER;

    -- Fleet Manager Database
    CREATE DATABASE atlasmesh_fleet;
    GRANT ALL PRIVILEGES ON DATABASE atlasmesh_fleet TO $POSTGRES_USER;

    -- Analytics Database
    CREATE DATABASE atlasmesh_analytics;
    GRANT ALL PRIVILEGES ON DATABASE atlasmesh_analytics TO $POSTGRES_USER;
EOSQL

echo "All databases created successfully!"
