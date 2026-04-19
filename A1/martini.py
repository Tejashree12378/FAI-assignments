import sys

def main():
    if len(sys.argv) != 2:
        print("Error: invalid arguments")
        return

    try:
        n = int(sys.argv[1])
    except ValueError:
        print("Error: invalid argument type")
        return

    if n <= 0:
        return

    width = n
    indent = 0

    # Build the bowl of the glass with '%' character
    while width > 0:
        print(" " * indent + "%" * width)
        indent += 1
        if width == 2:
            width = 1
            indent -= 1
        else:
            width -= 2

    if n % 2 == 0:
        stem_col = (n // 2) - 1
    else:
        stem_col = n // 2

    # Build the stem of the glass with '|' character
    for _ in range(n):
        print(" " * stem_col + "|")

    # Build the base of the glass with '=' character
    print("=" * n)

if __name__ == "__main__":
    main()
