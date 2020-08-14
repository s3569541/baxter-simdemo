xhost +

cat > .env << EOF
BAXTER_ONLY=false
XDISPLAY=novnc:0
EOF

docker-compose up
