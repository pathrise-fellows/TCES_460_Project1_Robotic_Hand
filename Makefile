schedule: server.c client.c continous_client.c continous_server.c
	gcc -o server server.c
	gcc -o client client.c
	gcc -o continous_client continous_client.c
	gcc -o continous_server continous_server.c
