



echo "NO FAILURE"
















for failure_type in {3..3}
do
	for motor in {6..6}
	do
	start_time=$(date +%s)
	echo "start time reached"
	echo "motor $motor ; failure_type $failure_type"
	python3 failure_simulation_with_chosen_failure.py  $motor $failure_type &
python_pid=$! 
	while true; do
		sleep_time=1
		sleep $sleep_time
		current_time=$(date +%s)
		elapsed_time=$((current_time - start_time))

		if [[ $elapsed_time -ge 20 ]]; then
			break
		fi
	done
	kill $python_pid
	zcho "COMPLETED motor $motor ; failure_type $failure_type"
	done
	
done
  




for failure_type in {0..2}
do
	for motor in {6..3}
	do
	start_time=$(date +%s)
	echo "start time reached"
	echo "motor $motor ; failure_type $failure_type"
	python3 failure_simulation_with_chosen_failure.py  $motor $failure_type &
python_pid=$!  
	
	while true; do
		sleep_time=1
		sleep $sleep_time
		current_time=$(date +%s)
		elapsed_time=$((current_time - start_time))

		if [[ $elapsed_time -ge 20 ]]; then
			break
		fi
	done
	kill $python_pid
	zcho "COMPLETED motor $motor ; failure_type $failure_type"
	done
	
done
  
