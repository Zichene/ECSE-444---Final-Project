def convert_html_to_c(html_var: str):
    with open("file.html", 'r') as file:
        lines = [line.rstrip() for line in file]

    with open('output.txt', 'a') as file:
        # first line is strcpy
        file.write(
            "strcpy((char *)http, (char *)\"HTTP/1.0 200 OK\\r\\nContent-Type: text/html\\r\\nPragma: "
            "no-cache\\r\\n\\r\\n\");\n")
        for line in lines:
            l = line.replace("\"", "\\\"")  # put a '\' in front of all " symbols
            file.write(f"strcat((char *){html_var}, (char *)\"{l.strip()}\\r\\n\");\n")

def clear_output_file():
    open('output.txt', 'w').close()
