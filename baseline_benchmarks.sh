source_dir=$HOME/"ECE565_final_project/Purdue-ECE-56500-project/m5out/"
source_file="stats.txt"
destination_dir=$HOME/"ECE565_final_project/Purdue-ECE-56500-project/benchmarks_results/baseline_minor"

while IFS= read -r variable; do
    echo "benchmark running: $variable" #| cut -d'.' -f2
    ./build/ECE565-ARM/gem5.opt configs/spec/spec_se.py -b $variable --cpu-type=MinorCPU --l1d_size=64kB --l1i_size=16kB --caches --l2cache --maxinsts=10000000 --lvp-type=LoadValuePredictor --cvu-type=ConstantVerificationUnit #| cut -d'.' -f2
    cp "$source_dir/$source_file" "$destination_dir"
    mv "$destination_dir/$source_file" "$destination_dir/stats_baseline_$variable.txt"
    # echo "source dir: $source_dir"
    grep -E "system.cpu.loadValuePred.predictedIncorrect|system.cpu.constantVU.loadMatched|system.cpu.loadValuePred.lvpuLookups|system.cpu.dcache.overallAvgMissLatency::total|system.cpu.ipc|simSeconds" $destination_dir/stats_baseline_$variable.txt > $destination_dir/grep_$variable.txt
    wait $!
done < "benchmarks_list.txt"
