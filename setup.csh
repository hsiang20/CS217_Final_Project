setenv REPO_TOP `realpath ./`
setenv SRC_HOME `realpath ./src/`
setenv HLS_HOME `realpath ./hls/`
setenv AWS_HOME `realpath ./design_top/`

echo "REPO_TOP is set to "`realpath $REPO_TOP`
echo "SRC_HOME is set to "`realpath $SRC_HOME`
echo "HLS_HOME is set to "`realpath $HLS_HOME`
echo "AWS_HOME is set to "`realpath $AWS_HOME`

module load $REPO_TOP/scripts/hls/catapult.module
setenv VG_GNU_PACKAGE /cad/synopsys/vcs_gnu_package/S-2021.09/gnu_9/linux
source /cad/synopsys/vcs_gnu_package/S-2021.09/gnu_9/linux/source_me.csh
