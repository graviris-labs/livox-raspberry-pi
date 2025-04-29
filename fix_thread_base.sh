#!/bin/bash
# This script adds the missing #include <memory> to thread_base.h

# Update the Dockerfile to add this script and run it during build
cat <<EOL > fix_thread_base.sh
#!/bin/bash
set -e

# Add missing include to thread_base.h
THREAD_BASE_H="/ros2_ws/src/livox_ros2_driver/livox_sdk_vendor/sdk_core/src/base/thread_base.h"
if [ -f "\$THREAD_BASE_H" ]; then
  # Check if memory is already included
  if ! grep -q "#include <memory>" "\$THREAD_BASE_H"; then
    # Add include after the include of noncopyable.h
    sed -i '/#include "noncopyable.h"/a #include <memory>' "\$THREAD_BASE_H"
    echo "Added #include <memory> to thread_base.h"
  else
    echo "thread_base.h already has #include <memory>"
  fi
else
  echo "thread_base.h not found at \$THREAD_BASE_H"
  exit 1
fi
EOL

chmod +x fix_thread_base.sh