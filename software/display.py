

def main():
    f = open('display.txt', 'r')
    while True:
        line = f.readline()
        print(line)

if __name__ == "__main__":
    main()
