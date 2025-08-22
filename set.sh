sudo cpupower frequency-set -g performance


echo 0-2,4-7 | sudo tee /proc/irq/163/smp_affinity_list
echo 0-2,4-7 | sudo tee /proc/irq/191/smp_affinity_list
echo 0-2,4-7 | sudo tee /proc/irq/196/smp_affinity_list
echo 0-2,4-7 | sudo tee /proc/irq/198/smp_affinity_list
echo 0-2,4-7 | sudo tee /proc/irq/197/smp_affinity_list
sudo systemctl stop irqbalance
sudo systemctl disable irqbalance
