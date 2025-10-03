#!/usr/bin/env python3
"""
AtlasMesh Fleet OS - Incident Response Automation Bot

This bot handles incident detection, escalation, and coordination across
PagerDuty, Slack, and other communication channels.
"""

import asyncio
import aiohttp
import json
import logging
import os
import time
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, asdict
from datetime import datetime, timedelta
from enum import Enum

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class Severity(Enum):
    """Incident severity levels"""
    CRITICAL = "critical"  # S1: Safety/compliance/complete outage
    HIGH = "high"         # S2: Major feature outage
    MEDIUM = "medium"     # S3: Performance degradation
    LOW = "low"          # S4: Minor issues with workarounds

class IncidentStatus(Enum):
    """Incident status"""
    OPEN = "open"
    ACKNOWLEDGED = "acknowledged"
    INVESTIGATING = "investigating"
    MITIGATING = "mitigating"
    RESOLVED = "resolved"
    CLOSED = "closed"

@dataclass
class Incident:
    """Incident data structure"""
    id: str
    title: str
    description: str
    severity: Severity
    status: IncidentStatus
    created_at: datetime
    updated_at: datetime
    assigned_to: Optional[str] = None
    service: Optional[str] = None
    alert_source: Optional[str] = None
    trace_id: Optional[str] = None
    runbook_url: Optional[str] = None
    slack_channel: Optional[str] = None
    pagerduty_incident_id: Optional[str] = None
    metrics: Optional[Dict] = None
    timeline: Optional[List[Dict]] = None

class PagerDutyClient:
    """PagerDuty API client"""
    
    def __init__(self, api_key: str, service_id: str):
        self.api_key = api_key
        self.service_id = service_id
        self.base_url = "https://api.pagerduty.com"
        self.session: Optional[aiohttp.ClientSession] = None
    
    async def __aenter__(self):
        headers = {
            'Authorization': f'Token token={self.api_key}',
            'Accept': 'application/vnd.pagerduty+json;version=2',
            'Content-Type': 'application/json'
        }
        self.session = aiohttp.ClientSession(headers=headers)
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        if self.session:
            await self.session.close()
    
    async def create_incident(self, incident: Incident) -> Optional[str]:
        """Create incident in PagerDuty"""
        try:
            payload = {
                "incident": {
                    "type": "incident",
                    "title": incident.title,
                    "service": {
                        "id": self.service_id,
                        "type": "service_reference"
                    },
                    "urgency": "high" if incident.severity in [Severity.CRITICAL, Severity.HIGH] else "low",
                    "body": {
                        "type": "incident_body",
                        "details": incident.description
                    }
                }
            }
            
            async with self.session.post(f"{self.base_url}/incidents", json=payload) as response:
                if response.status == 201:
                    data = await response.json()
                    pagerduty_id = data['incident']['id']
                    logger.info(f"Created PagerDuty incident: {pagerduty_id}")
                    return pagerduty_id
                else:
                    logger.error(f"Failed to create PagerDuty incident: {response.status}")
                    return None
                    
        except Exception as e:
            logger.error(f"PagerDuty API error: {str(e)}")
            return None
    
    async def update_incident(self, pagerduty_id: str, status: str, note: str = None) -> bool:
        """Update incident status in PagerDuty"""
        try:
            payload = {
                "incident": {
                    "type": "incident",
                    "status": status
                }
            }
            
            if note:
                payload["incident"]["resolution"] = note
            
            async with self.session.put(f"{self.base_url}/incidents/{pagerduty_id}", json=payload) as response:
                if response.status == 200:
                    logger.info(f"Updated PagerDuty incident {pagerduty_id}: {status}")
                    return True
                else:
                    logger.error(f"Failed to update PagerDuty incident: {response.status}")
                    return False
                    
        except Exception as e:
            logger.error(f"PagerDuty update error: {str(e)}")
            return False

class SlackClient:
    """Slack API client"""
    
    def __init__(self, bot_token: str, app_token: str):
        self.bot_token = bot_token
        self.app_token = app_token
        self.base_url = "https://slack.com/api"
        self.session: Optional[aiohttp.ClientSession] = None
    
    async def __aenter__(self):
        headers = {
            'Authorization': f'Bearer {self.bot_token}',
            'Content-Type': 'application/json'
        }
        self.session = aiohttp.ClientSession(headers=headers)
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        if self.session:
            await self.session.close()
    
    async def create_incident_channel(self, incident: Incident) -> Optional[str]:
        """Create dedicated Slack channel for incident"""
        try:
            channel_name = f"incident-{incident.id.lower()}"
            
            # Create channel
            payload = {
                "name": channel_name,
                "is_private": False
            }
            
            async with self.session.post(f"{self.base_url}/conversations.create", json=payload) as response:
                if response.status == 200:
                    data = await response.json()
                    if data.get('ok'):
                        channel_id = data['channel']['id']
                        logger.info(f"Created Slack channel: #{channel_name}")
                        
                        # Set channel topic
                        await self.set_channel_topic(channel_id, f"🚨 {incident.title} | Severity: {incident.severity.value.upper()}")
                        
                        # Post initial incident details
                        await self.post_incident_message(channel_id, incident)
                        
                        return channel_id
                    else:
                        logger.error(f"Slack API error: {data.get('error')}")
                        return None
                else:
                    logger.error(f"Failed to create Slack channel: {response.status}")
                    return None
                    
        except Exception as e:
            logger.error(f"Slack channel creation error: {str(e)}")
            return None
    
    async def set_channel_topic(self, channel_id: str, topic: str) -> bool:
        """Set Slack channel topic"""
        try:
            payload = {
                "channel": channel_id,
                "topic": topic
            }
            
            async with self.session.post(f"{self.base_url}/conversations.setTopic", json=payload) as response:
                data = await response.json()
                return data.get('ok', False)
                
        except Exception as e:
            logger.error(f"Slack topic update error: {str(e)}")
            return False
    
    async def post_incident_message(self, channel_id: str, incident: Incident) -> bool:
        """Post incident details to Slack channel"""
        try:
            # Create rich message with incident details
            blocks = [
                {
                    "type": "header",
                    "text": {
                        "type": "plain_text",
                        "text": f"🚨 Incident: {incident.title}"
                    }
                },
                {
                    "type": "section",
                    "fields": [
                        {
                            "type": "mrkdwn",
                            "text": f"*Severity:* {incident.severity.value.upper()}"
                        },
                        {
                            "type": "mrkdwn",
                            "text": f"*Status:* {incident.status.value.title()}"
                        },
                        {
                            "type": "mrkdwn",
                            "text": f"*Service:* {incident.service or 'Unknown'}"
                        },
                        {
                            "type": "mrkdwn",
                            "text": f"*Created:* {incident.created_at.strftime('%Y-%m-%d %H:%M:%S UTC')}"
                        }
                    ]
                },
                {
                    "type": "section",
                    "text": {
                        "type": "mrkdwn",
                        "text": f"*Description:*\n{incident.description}"
                    }
                }
            ]
            
            # Add runbook link if available
            if incident.runbook_url:
                blocks.append({
                    "type": "section",
                    "text": {
                        "type": "mrkdwn",
                        "text": f"📖 *Runbook:* <{incident.runbook_url}|View Runbook>"
                    }
                })
            
            # Add trace ID if available
            if incident.trace_id:
                blocks.append({
                    "type": "section",
                    "text": {
                        "type": "mrkdwn",
                        "text": f"🔍 *Trace ID:* `{incident.trace_id}`"
                    }
                })
            
            # Add action buttons
            blocks.append({
                "type": "actions",
                "elements": [
                    {
                        "type": "button",
                        "text": {
                            "type": "plain_text",
                            "text": "Acknowledge"
                        },
                        "style": "primary",
                        "value": f"ack_{incident.id}",
                        "action_id": "acknowledge_incident"
                    },
                    {
                        "type": "button",
                        "text": {
                            "type": "plain_text",
                            "text": "Escalate"
                        },
                        "style": "danger",
                        "value": f"escalate_{incident.id}",
                        "action_id": "escalate_incident"
                    }
                ]
            })
            
            payload = {
                "channel": channel_id,
                "blocks": blocks
            }
            
            async with self.session.post(f"{self.base_url}/chat.postMessage", json=payload) as response:
                data = await response.json()
                return data.get('ok', False)
                
        except Exception as e:
            logger.error(f"Slack message posting error: {str(e)}")
            return False
    
    async def post_update(self, channel_id: str, message: str, thread_ts: str = None) -> bool:
        """Post update to incident channel"""
        try:
            payload = {
                "channel": channel_id,
                "text": message,
                "thread_ts": thread_ts
            }
            
            async with self.session.post(f"{self.base_url}/chat.postMessage", json=payload) as response:
                data = await response.json()
                return data.get('ok', False)
                
        except Exception as e:
            logger.error(f"Slack update posting error: {str(e)}")
            return False

class StatusPageClient:
    """Status page API client"""
    
    def __init__(self, api_key: str, page_id: str):
        self.api_key = api_key
        self.page_id = page_id
        self.base_url = "https://api.statuspage.io/v1"
        self.session: Optional[aiohttp.ClientSession] = None
    
    async def __aenter__(self):
        headers = {
            'Authorization': f'OAuth {self.api_key}',
            'Content-Type': 'application/json'
        }
        self.session = aiohttp.ClientSession(headers=headers)
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        if self.session:
            await self.session.close()
    
    async def create_incident(self, incident: Incident) -> Optional[str]:
        """Create incident on status page"""
        try:
            # Map severity to status page impact
            impact_map = {
                Severity.CRITICAL: "critical",
                Severity.HIGH: "major",
                Severity.MEDIUM: "minor",
                Severity.LOW: "minor"
            }
            
            payload = {
                "incident": {
                    "name": incident.title,
                    "status": "investigating",
                    "impact_override": impact_map[incident.severity],
                    "body": incident.description
                }
            }
            
            async with self.session.post(f"{self.base_url}/pages/{self.page_id}/incidents", json=payload) as response:
                if response.status == 201:
                    data = await response.json()
                    status_page_id = data['id']
                    logger.info(f"Created status page incident: {status_page_id}")
                    return status_page_id
                else:
                    logger.error(f"Failed to create status page incident: {response.status}")
                    return None
                    
        except Exception as e:
            logger.error(f"Status page API error: {str(e)}")
            return None

class IncidentResponseBot:
    """Main incident response automation bot"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.incidents: Dict[str, Incident] = {}
        
        # Initialize clients
        self.pagerduty = PagerDutyClient(
            config['pagerduty']['api_key'],
            config['pagerduty']['service_id']
        ) if config.get('pagerduty') else None
        
        self.slack = SlackClient(
            config['slack']['bot_token'],
            config['slack']['app_token']
        ) if config.get('slack') else None
        
        self.status_page = StatusPageClient(
            config['status_page']['api_key'],
            config['status_page']['page_id']
        ) if config.get('status_page') else None
    
    def generate_incident_id(self) -> str:
        """Generate unique incident ID"""
        timestamp = datetime.utcnow().strftime("%Y%m%d-%H%M%S")
        return f"INC-{timestamp}"
    
    async def create_incident(self, title: str, description: str, severity: Severity, 
                            service: str = None, alert_source: str = None, 
                            trace_id: str = None) -> Incident:
        """Create new incident and trigger response workflow"""
        
        incident_id = self.generate_incident_id()
        now = datetime.utcnow()
        
        # Determine runbook URL based on service/alert
        runbook_url = self._get_runbook_url(service, alert_source)
        
        incident = Incident(
            id=incident_id,
            title=title,
            description=description,
            severity=severity,
            status=IncidentStatus.OPEN,
            created_at=now,
            updated_at=now,
            service=service,
            alert_source=alert_source,
            trace_id=trace_id,
            runbook_url=runbook_url,
            timeline=[{
                "timestamp": now.isoformat(),
                "event": "incident_created",
                "message": "Incident created by automated system"
            }]
        )
        
        self.incidents[incident_id] = incident
        
        logger.info(f"Created incident {incident_id}: {title} (Severity: {severity.value})")
        
        # Trigger response workflow
        await self._trigger_incident_response(incident)
        
        return incident
    
    def _get_runbook_url(self, service: str, alert_source: str) -> Optional[str]:
        """Get runbook URL based on service and alert"""
        runbook_base = "https://docs.atlasmesh.ae/runbooks"
        
        if service:
            return f"{runbook_base}/{service.lower().replace('-', '_')}"
        elif alert_source:
            return f"{runbook_base}/{alert_source.lower().replace('-', '_')}"
        else:
            return f"{runbook_base}/general"
    
    async def _trigger_incident_response(self, incident: Incident):
        """Trigger incident response workflow"""
        try:
            # Create PagerDuty incident
            if self.pagerduty:
                async with self.pagerduty as pd:
                    pagerduty_id = await pd.create_incident(incident)
                    if pagerduty_id:
                        incident.pagerduty_incident_id = pagerduty_id
            
            # Create Slack incident channel
            if self.slack:
                async with self.slack as slack:
                    channel_id = await slack.create_incident_channel(incident)
                    if channel_id:
                        incident.slack_channel = channel_id
            
            # Create status page incident for critical/high severity
            if self.status_page and incident.severity in [Severity.CRITICAL, Severity.HIGH]:
                async with self.status_page as sp:
                    await sp.create_incident(incident)
            
            # Update incident timeline
            incident.timeline.append({
                "timestamp": datetime.utcnow().isoformat(),
                "event": "response_triggered",
                "message": "Incident response workflow triggered"
            })
            
            logger.info(f"Incident response triggered for {incident.id}")
            
        except Exception as e:
            logger.error(f"Failed to trigger incident response: {str(e)}")
    
    async def update_incident(self, incident_id: str, status: IncidentStatus, 
                            message: str, assigned_to: str = None) -> bool:
        """Update incident status and notify stakeholders"""
        
        if incident_id not in self.incidents:
            logger.error(f"Incident {incident_id} not found")
            return False
        
        incident = self.incidents[incident_id]
        old_status = incident.status
        
        incident.status = status
        incident.updated_at = datetime.utcnow()
        
        if assigned_to:
            incident.assigned_to = assigned_to
        
        # Add to timeline
        incident.timeline.append({
            "timestamp": incident.updated_at.isoformat(),
            "event": f"status_changed_{status.value}",
            "message": message,
            "assigned_to": assigned_to
        })
        
        logger.info(f"Updated incident {incident_id}: {old_status.value} -> {status.value}")
        
        # Notify stakeholders
        await self._notify_incident_update(incident, message)
        
        return True
    
    async def _notify_incident_update(self, incident: Incident, message: str):
        """Notify stakeholders of incident update"""
        try:
            # Update PagerDuty
            if self.pagerduty and incident.pagerduty_incident_id:
                status_map = {
                    IncidentStatus.ACKNOWLEDGED: "acknowledged",
                    IncidentStatus.INVESTIGATING: "acknowledged",
                    IncidentStatus.MITIGATING: "acknowledged",
                    IncidentStatus.RESOLVED: "resolved"
                }
                
                if incident.status in status_map:
                    async with self.pagerduty as pd:
                        await pd.update_incident(
                            incident.pagerduty_incident_id,
                            status_map[incident.status],
                            message
                        )
            
            # Update Slack channel
            if self.slack and incident.slack_channel:
                async with self.slack as slack:
                    update_message = f"📢 **Status Update:** {incident.status.value.title()}\n{message}"
                    await slack.post_update(incident.slack_channel, update_message)
            
        except Exception as e:
            logger.error(f"Failed to notify incident update: {str(e)}")
    
    async def handle_prometheus_alert(self, alert_data: Dict) -> Optional[Incident]:
        """Handle incoming Prometheus alert"""
        try:
            # Parse alert data
            alert_name = alert_data.get('alertname', 'Unknown Alert')
            description = alert_data.get('annotations', {}).get('description', 'No description')
            severity_str = alert_data.get('labels', {}).get('severity', 'medium')
            service = alert_data.get('labels', {}).get('service', 'unknown')
            
            # Map severity
            severity_map = {
                'critical': Severity.CRITICAL,
                'high': Severity.HIGH,
                'warning': Severity.MEDIUM,
                'info': Severity.LOW
            }
            severity = severity_map.get(severity_str, Severity.MEDIUM)
            
            # Check if we should create an incident (not all alerts need incidents)
            if severity in [Severity.CRITICAL, Severity.HIGH]:
                incident = await self.create_incident(
                    title=f"Alert: {alert_name}",
                    description=description,
                    severity=severity,
                    service=service,
                    alert_source="prometheus"
                )
                return incident
            else:
                logger.info(f"Alert {alert_name} (severity: {severity_str}) - no incident created")
                return None
                
        except Exception as e:
            logger.error(f"Failed to handle Prometheus alert: {str(e)}")
            return None
    
    def get_incident_summary(self) -> Dict:
        """Get summary of all incidents"""
        now = datetime.utcnow()
        
        active_incidents = [i for i in self.incidents.values() 
                          if i.status not in [IncidentStatus.RESOLVED, IncidentStatus.CLOSED]]
        
        summary = {
            "total_incidents": len(self.incidents),
            "active_incidents": len(active_incidents),
            "by_severity": {
                "critical": len([i for i in active_incidents if i.severity == Severity.CRITICAL]),
                "high": len([i for i in active_incidents if i.severity == Severity.HIGH]),
                "medium": len([i for i in active_incidents if i.severity == Severity.MEDIUM]),
                "low": len([i for i in active_incidents if i.severity == Severity.LOW])
            },
            "by_status": {
                status.value: len([i for i in active_incidents if i.status == status])
                for status in IncidentStatus
            },
            "recent_incidents": [
                {
                    "id": i.id,
                    "title": i.title,
                    "severity": i.severity.value,
                    "status": i.status.value,
                    "created_at": i.created_at.isoformat(),
                    "service": i.service
                }
                for i in sorted(self.incidents.values(), key=lambda x: x.created_at, reverse=True)[:10]
            ]
        }
        
        return summary

async def main():
    """Main execution function for testing"""
    # Example configuration
    config = {
        "pagerduty": {
            "api_key": os.getenv("PAGERDUTY_API_KEY", "dummy_key"),
            "service_id": os.getenv("PAGERDUTY_SERVICE_ID", "dummy_service")
        },
        "slack": {
            "bot_token": os.getenv("SLACK_BOT_TOKEN", "dummy_token"),
            "app_token": os.getenv("SLACK_APP_TOKEN", "dummy_token")
        },
        "status_page": {
            "api_key": os.getenv("STATUSPAGE_API_KEY", "dummy_key"),
            "page_id": os.getenv("STATUSPAGE_PAGE_ID", "dummy_page")
        }
    }
    
    # Initialize incident response bot
    bot = IncidentResponseBot(config)
    
    # Example: Create test incident
    incident = await bot.create_incident(
        title="Test Incident - Fleet Manager High Error Rate",
        description="Fleet Manager service is experiencing error rate above 5% threshold",
        severity=Severity.HIGH,
        service="fleet-manager",
        trace_id="abc123def456"
    )
    
    logger.info(f"Created test incident: {incident.id}")
    
    # Example: Update incident
    await asyncio.sleep(2)
    await bot.update_incident(
        incident.id,
        IncidentStatus.ACKNOWLEDGED,
        "Incident acknowledged by on-call engineer",
        assigned_to="engineer@atlasmesh.ae"
    )
    
    # Get summary
    summary = bot.get_incident_summary()
    print(json.dumps(summary, indent=2))

if __name__ == '__main__':
    asyncio.run(main())
