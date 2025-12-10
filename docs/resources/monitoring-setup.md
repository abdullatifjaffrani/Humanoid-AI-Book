---
title: Monitoring and Uptime Checks Setup
sidebar_position: 9
---

# Monitoring and Uptime Checks Setup

This guide provides instructions for setting up monitoring and uptime checks for the Physical AI & Humanoid Robotics Textbook website to ensure optimal performance and availability.

## Uptime Monitoring

### Third-Party Monitoring Services
Consider using these services for uptime monitoring:

#### UptimeRobot
- Free tier supports 50 monitors
- Supports HTTP/HTTPS monitoring
- Email and Slack notifications
- Public status page capability

Configuration steps:
1. Create account at uptimerobot.com
2. Add new monitor for your site URL
3. Set monitoring interval (5 minutes for free tier)
4. Configure notification settings
5. Set up public status page if needed

#### Pingdom
- Comprehensive monitoring solution
- Real user monitoring
- Transaction monitoring
- Performance insights

#### StatusCake
- Affordable monitoring solution
- Multiple location monitoring
- SSL certificate monitoring
- Speed testing

### Custom Monitoring Scripts

Create a simple uptime check script:

```bash
#!/bin/bash
# uptime-check.sh
URL="https://humanide-ai.github.io/Humanide-AI-Book"
LOG_FILE="/var/log/uptime-check.log"

# Check if site is accessible
HTTP_CODE=$(curl -o /dev/null -s -w "%{http_code}" $URL)

if [ $HTTP_CODE -eq 200 ]; then
    echo "$(date): Site is UP (HTTP $HTTP_CODE)" >> $LOG_FILE
    # Optional: send success notification
else
    echo "$(date): Site is DOWN (HTTP $HTTP_CODE)" >> $LOG_FILE
    # Send alert notification
    # Example: curl -X POST -H "Content-Type: application/json" -d '{"text":"Site is down!"}' $SLACK_WEBHOOK_URL
fi
```

## Performance Monitoring

### Google Analytics
Set up Google Analytics for user behavior tracking:

1. Create Google Analytics account
2. Set up property for your website
3. Get the GA tracking ID
4. Add tracking code to Docusaurus config

In `docusaurus.config.js`:
```javascript
presets: [
  [
    'classic',
    {
      // ... other config
      gtag: {
        trackingID: 'GA-TRACKING-ID',
        anonymizeIP: true,
      },
    },
  ],
],
```

### Performance Monitoring Tools

#### Google PageSpeed Insights API
Automate performance monitoring:

```javascript
// pagespeed-check.js
const axios = require('axios');

async function checkPageSpeed(url) {
  const response = await axios.get(
    `https://www.googleapis.com/pagespeedonline/v5/runPagespeed?url=${url}`
  );

  const data = response.data;
  console.log(`Performance Score: ${data.lighthouseResult.categories.performance.score * 100}`);
  console.log(`Loading Time: ${data.lighthouseResult.audits.metrics.details.items[0].interactive}ms`);
}
```

## Server-Side Monitoring

### GitHub Pages Monitoring
Since the site is hosted on GitHub Pages:

1. Monitor GitHub status at https://www.githubstatus.com/
2. Set up GitHub service hooks for deployment notifications
3. Use external monitoring services for uptime

### Content Delivery Network (CDN) Considerations
GitHub Pages uses a CDN. Monitor for:
- Cache invalidation issues
- Geographic performance variations
- Asset loading times

## Alert Configuration

### Alert Thresholds
Configure alerts with appropriate thresholds:

- Uptime: Alert if down for more than 1 minute
- Response time: Alert if > 5 seconds consistently
- Error rate: Alert if > 5% of requests fail
- SSL certificate: Alert 30 days before expiration

### Notification Channels
Set up multiple notification channels:

1. **Email**: For all critical alerts
2. **Slack/Discord**: For team notifications
3. **SMS**: For critical issues requiring immediate attention
4. **GitHub Issues**: For tracking recurring problems

## Automated Maintenance Checks

### SSL Certificate Monitoring
Create a script to monitor SSL certificate expiration:

```bash
#!/bin/bash
# ssl-check.sh
DOMAIN="humanide-ai.github.io"
DAYS_LEFT=$(echo | openssl s_client -connect $DOMAIN:443 2>/dev/null | openssl x509 -noout -days_valid)

if [ $DAYS_LEFT -lt 30 ]; then
    echo "SSL certificate for $DOMAIN expires in $DAYS_LEFT days!"
    # Send alert
fi
```

### Content Freshness Checks
Monitor for stale content:

```javascript
// content-freshness.js
const fs = require('fs');
const path = require('path');

function checkContentFreshness(docsPath) {
  const files = fs.readdirSync(docsPath);
  const thirtyDaysAgo = new Date();
  thirtyDaysAgo.setDate(thirtyDaysAgo.getDate() - 30);

  files.forEach(file => {
    const stats = fs.statSync(path.join(docsPath, file));
    if (stats.mtime < thirtyDaysAgo) {
      console.log(`File ${file} hasn't been updated in 30+ days`);
    }
  });
}
```

## Monitoring Dashboard Setup

### Key Metrics to Track
- **Uptime percentage** (target: 99.9%)
- **Average response time** (target: < 2 seconds)
- **Page load performance** (target: < 3 seconds)
- **Error rate** (target: < 1%)
- **Traffic analytics** (visitors, page views, bounce rate)

### Dashboard Tools
- **Grafana**: For custom dashboards
- **Google Analytics Dashboard**: For user analytics
- **GitHub Insights**: For repository metrics
- **Custom scripts**: For specific metrics

## Incident Response

### Response Procedures
1. Acknowledge alert immediately
2. Determine scope of impact
3. Check GitHub status for platform issues
4. Investigate root cause
5. Implement fix or workaround
6. Verify resolution
7. Document incident for future reference

### Escalation Process
- Level 1: Automated alerts to development team
- Level 2: If unresolved after 30 minutes, escalate to senior team
- Level 3: If unresolved after 2 hours, contact hosting provider

## Maintenance Scheduling

### Regular Maintenance Tasks
- Weekly: Review monitoring logs and alerts
- Monthly: Analyze performance trends
- Quarterly: Review and update monitoring configuration
- Annually: Review monitoring service needs and costs

### Maintenance Windows
Schedule maintenance during low-traffic periods:
- Time: 2:00 AM - 4:00 AM server time
- Frequency: Monthly
- Duration: Maximum 2 hours
- Notification: 24 hours in advance

## Reporting

### Regular Reports
- Weekly uptime reports
- Monthly performance summaries
- Quarterly availability reports
- Annual monitoring effectiveness review

### Report Contents
- Uptime statistics
- Performance metrics
- Alert frequency and resolution times
- Cost analysis of monitoring services
- Recommendations for improvements

This monitoring setup will help ensure the Physical AI & Humanoid Robotics Textbook remains available, performant, and reliable for all users.