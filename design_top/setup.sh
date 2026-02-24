#
# Copyright 2026 Stanford University
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

export CL_DESIGN_NAME=design_top
export CL_DESIGN_DESCRIPTION='lab4 Accelerator Top'
export CL_DIR=$(pwd)

export DONT_GENERATE_FILE_LIST=1

generate_afi () {
    export CL_DESIGN_NAME=counter
    export CL_DESIGN_DESCRIPTION='lab4 Accelerator Tope'
    export CL_DIR=$(pwd)
    FILENAME=$(ls -lrt build/checkpoints/*.tar | tail -n 1 | awk '{print $NF}')
    export DCP_TARBALL_TO_INGEST=$(realpath "$FILENAME")
    export DCP_TARBALL_NAME=$(basename ${DCP_TARBALL_TO_INGEST})
    aws s3 cp ${DCP_TARBALL_TO_INGEST} s3://${DCP_BUCKET_NAME}/${DCP_FOLDER_NAME}/
    OUTPUT=$(aws ec2 create-fpga-image --name ${CL_DESIGN_NAME} --description "${CL_DESIGN_DESCRIPTION}" --input-storage-location Bucket=${DCP_BUCKET_NAME},Key=${DCP_FOLDER_NAME}/${DCP_TARBALL_NAME} --logs-storage-location Bucket=${LOGS_BUCKET_NAME},Key=${LOGS_FOLDER_NAME}/ --region ${REGION})
    echo $OUTPUT
    export AFI=$(echo "$OUTPUT" | jq -r '.FpgaImageId')
    export AGFI=$(echo "$OUTPUT" | jq -r '.FpgaImageGlobalId')
    echo "export AFI=$AFI" >> generated_afid.sh
    echo "export AGFI=$AGFI" >> generated_afid.sh
    echo "AFI is set to: $AFI"
    echo "AGFI is set to: $AGFI"
}