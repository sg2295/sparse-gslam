# Loop over all topics, printing any that has no active subscribers
for topic in $(rostopic list); do
  num_subs=$(rostopic info $topic | grep -c "Subscribers: None");
  if [ $num_subs -gt 0 ]; then
    echo $topic;
  fi;
done
