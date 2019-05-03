function listener(){
		var listener = new ROSLIB.Topic({
		  ros : ros,
		  name : '/chatter',
		  messageType : 'std_msgs/String'
		});

		// Then we add a callback to be called every time a message is published on this topic.
		listener.subscribe(function(message) {
		  console.log('Received message on ' + listener.name + ': ' + message.data);

		  // If desired, we can unsubscribe from the topic as well.
		  listener.unsubscribe();
		});
	}
