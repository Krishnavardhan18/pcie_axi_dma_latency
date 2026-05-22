set_param project.enableReportConfiguration 0
load_feature core
current_fileset
xsim {tb_pcie_axi_dma_snap} -wdb {/get/work/krishna.vardhan/git_trees/mtp_pcie_axi/build/tb_pcie_axi_dma.wdb} -autoloadwcfg -tclbatch {/dev/null}
