def add_line_to_file(file_path, line):
    """Adds a line to a text file.

    Args:
        file_path: The path to the text file.
        line: The string to add to the file.
    """
    try:
        with open(file_path, 'a') as file:
            file.write(line + '\n')
    except FileNotFoundError:
        print(f"Error: File not found at {file_path}")
    except Exception as e:
        print(f"An error occurred: {e}")
