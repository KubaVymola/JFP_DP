import sys
import pandas as pd

def main():
    df = pd.read_csv(sys.argv[1])

    print(df.std())

if __name__ == '__main__':
    main()