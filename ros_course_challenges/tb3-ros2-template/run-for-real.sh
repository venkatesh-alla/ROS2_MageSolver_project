if [ -z "$1" ]; then
    echo "Usage: $0 ROS_DOMAIN_ID"
    exit 1
fi
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=$1
python3 tb3.py
