# Lexx500 Wi-Fi Heatmapping Tool

Wi-Fi survey and heatmapping tool for Lexx500 AMRs.

## Features
- RSSI median and P10 heatmaps
- Bad-zone detection
- Path overlay with roam / disconnect markers
- RSSI time series and histogram
- Self-contained HTML report

## Typical workflow
1. Collect data while walking or driving a route
2. Render heatmaps and HTML report
3. Use P10 + bad-zone analysis to guide tuning or AP placement

## Requirements
- Python 3.10+
- pandas, numpy, matplotlib, pillow

## Notes
- Designed for MOXA-1173C (V6) and FXE-5000 (V7)
- Robot-side integration via ROS planned
