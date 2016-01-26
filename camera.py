from time import sleep

def main():
    f = open("/var/log/test.log",'w')
    while True:
        f.write("poop\n")
        sleep(0.1)
        f.flush()
        pass

if __name__ == "__main__":
    main()
