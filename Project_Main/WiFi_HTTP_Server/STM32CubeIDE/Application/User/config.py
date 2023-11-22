class ConfigObj:
    def __init__(self, board_receiving: bool, ssid: str, password: str, security: str, host_ip: list):
        self.board_receiving = board_receiving
        self.ssid = ssid
        self.password = password
        self.security = security
        self.host_ip = host_ip


def read_config():
    with open('boards.cfg', 'r') as file:
        lines = [line.rstrip() for line in file if '#' not in line]
        file.close()

    if not check_fields(lines):
        return None

    if 'true' in lines[0]:
        board_receiving = True
    elif 'false' in lines[0]:
        board_receiving = False
    else:
        print("ERROR: wrong argument for board_receiving")
        return None

    ssid = lines[1].replace('ssid = ', '')
    password = lines[2].replace('password = ', '')
    security = lines[3].replace('security = ', '')
    if (security != "WPA_PSK") and (security != 'WPA2_PSK') and (security != "OPEN"):
        print(f"ERROR: Wrong value \"{security}\" for field security")
        return None
    host_ip = convert_ip_str_to_array(lines[4].replace('host_ip = ', ''))
    return ConfigObj(board_receiving, ssid, password, security, host_ip)


def config(config_obj: ConfigObj):
    # create empty files if they are not there
    with open('receiving_board_config.h', 'w') as file:
        file.close()
    with open('sending_board_config.h', 'w') as file:
        file.close()
    lines = [f"#define SSID \t \"{config_obj.ssid}\"\n", f"#define PASSWORD \t \"{config_obj.password}\"\n"]
    security = "WIFI_ECN_" + config_obj.security
    lines.append(f"#define SECURITY \t {security}\n")
    lines_receiving = lines.copy()
    lines_sending = lines.copy()
    if config_obj.board_receiving:
        lines_receiving.append("#define RECEIVING_ACTIVE\n")
    else:
        # need to put IP info for sending board
        lines_sending.append(f"#define REMOTE_IP_0 \t {config_obj.host_ip[0]}\n")
        lines_sending.append(f"#define REMOTE_IP_1 \t {config_obj.host_ip[1]}\n")
        lines_sending.append(f"#define REMOTE_IP_2 \t {config_obj.host_ip[2]}\n")
        lines_sending.append(f"#define REMOTE_IP_3 \t {config_obj.host_ip[3]}\n")
        lines_sending.append("#define SENDING_ACTIVE\n")
    with open('receiving_board_config.h', 'w') as file:
        file.writelines(lines_receiving)
        file.close()
    with open('sending_board_config.h', 'w') as file:
        file.writelines(lines_sending)
        file.close()


def check_fields(conf_lines):
    # assuming that fields are in order
    if 'board_receiving' not in conf_lines[0]:
        print("ERROR: Missing field board_receiving, check conf file.")
        return False
    if 'ssid' not in conf_lines[1]:
        print("ERROR: Missing field ssid, check conf file.")
        return False
    if 'password' not in conf_lines[2]:
        print("ERROR: Missing field password, check conf file.")
        return False
    if 'security' not in conf_lines[3]:
        print("ERROR: Missing field security, check conf file.")
        return False
    if 'host_ip' not in conf_lines[4]:
        print("ERROR: Missing field host_ip, check conf file.")
        return False
    return True


def convert_ip_str_to_array(ip_str: str):
    ip_str = ip_str.replace('.', ' ')
    return [int(i) for i in ip_str.split()]


if __name__ == '__main__':
    ConfigObj = read_config()
    if ConfigObj is not None:
        config(ConfigObj)
        print("Sucessfully set up config header files")
    else:
        print("ERROR: Could not configure files")
