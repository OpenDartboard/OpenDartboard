#!/bin/bash
# Export OpenCV and other includes from the docker container to the local machine

# Create a directory for the exported includes
mkdir -p .vscode/container-includes

# Run the docker container and copy the OpenCV and ncnn includes
docker run --rm -v $PWD/.vscode/container-includes:/export opendartboard:latest bash -c '
mkdir -p /export/opencv2 /export/ncnn
cp -r /usr/include/opencv4/opencv2 /export/ 
cp -r /usr/local/include/ncnn /export/
'

echo "Include files exported to .vscode/container-includes"
echo "Update your c_cpp_properties.json to use these includes"
