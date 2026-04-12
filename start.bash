gnome-terminal --title "Script 1" -- bash -c "./start1.bash; exec bash" &
gnome-terminal --title "Script 2" -- bash -c "./start2.bash; exec bash" &
gnome-terminal --title "Script 3" -- bash -c "./start3.bash; exec bash" &



# ping 192.168.1.19   机械臂的IP
# ping 192.168.11.210  灵巧手的IP