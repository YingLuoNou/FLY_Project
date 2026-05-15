
# restart HNS services
net stop hns
net start hns

# reset WSL network configurations
netsh winsock reset
netsh int ip reset

# restart host computer, and set WSL network to ``mirrored'' mode.
