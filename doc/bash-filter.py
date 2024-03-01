import sys
import os
import re

__version__ = '1.0.0'

cmd_cls = 'keyword'
subcmd_cls = 'keywordtype'
comment_class = 'comment'
sudo_cmd_class = 'keywordflow'

# Bash commands that are generally followed by a second command
super_cmds = ['git', 'apt', 'docker']
url_pattern = r'(https?|ftp)://[^\s/$.?#].[^\s]*'

def ProcessWord(word: str, is_command = False, is_subcommand = False) -> str:
    """ Process word and return type

    :param word: word
    :param is_command: is command
    :param is_subcommand: is subcommand
    :return: type
    """
    if word == 'sudo' and is_command:
        print('<span class="{}">{}</span>'.format(sudo_cmd_class, word), end='')
        return "sudo"
    if re.match(url_pattern, word):
        print('<a href="{}">{}</a>'.format(word, word), end='')
        return "url"
    if word in super_cmds and is_command:
        print('<span class="{}">{}</span>'.format(cmd_cls, word), end='')
        return "super"
    if is_command:
        print('<span class="{}">{}</span>'.format(cmd_cls, word), end='')
        return
    if is_subcommand:
        print('<span class="{}">{}</span>'.format(subcmd_cls, word), end='')
        return
    print(word, end='')
    return

def ProcessBashCode(code: str) -> None:
    """ Process bash code and convert to doxygen format

    :param code: bash code
    :return: None
    """
    print('@htmlonly' + '\n', end='')
    # Start div fragment
    print('<div class="fragment">' + '\n', end='')

    # Split code into lines
    lines = code.split('\n')

    prev_line_continuation = False
    for line in lines:
        if len(line) == 0:
            continue
        # Start div line
        print('<div class="line">', end='')
        if prev_line_continuation:
            next_is_command = False
            next_is_subcommand = False
        else:
            next_is_command = True
            next_is_subcommand = False
        # if last character is \ then the command is continued in next line
        if line[-1] == '\\':
            prev_line_continuation = True
        else:
            prev_line_continuation = False

        word = ''
        char_count = 0
        for ch in line:
            char_count += 1
            if ch == '<':
                word += '&lt;'
                continue
            if ch == '>':
                word += '&gt;'
                continue
            if ch == '#':
                print('<span class="{}">{}</span>'.format(comment_class, line[char_count-1:]), end='')
                break
            if ch == ' ' or ch == '\t':
                if len(word) > 0:
                    word_type = ProcessWord(word, next_is_command, next_is_subcommand)
                    if word_type == "sudo":
                        next_is_command = True
                        next_is_subcommand = False
                    elif word_type == "super":
                        next_is_command = False
                        next_is_subcommand = True
                    else:
                        next_is_command = False
                        next_is_subcommand = False
                if ch == ' ':
                    print(ch, end='')
                elif ch == '\t':
                    print('&nbsp;&nbsp;&nbsp;&nbsp;', end='')
                word = ''
            else:
                word += ch

        if len(word) > 0:
            ProcessWord(word, next_is_command, next_is_subcommand)

        print('</div>' + '\n', end='')

    print('</div>' + '\n', end='')
    print('@endhtmlonly', end='')



def FileHandle(filehandle: type(open)) -> None:
    """ Read file handle and convert bash code to a doxygen convenient format.

    :param filehandle: input file handle
    :return: None
    """
    # Find all code blocks with bash code
    for line in filehandle:
        code_block = ""
        if re.match(r'^```bash', line):
            for line in filehandle:
                if re.match(r'^```', line):
                    break
                code_block += line
            ProcessBashCode(code_block)
        else:
            print(line, end='')
    return


def main(file: str) -> None:
    """ filter main procedure

    :param file: input file name or '-' for input stream
    :return:
    """
    if file == '-':
        FileHandle(sys.stdin)
    else:
        try:
            with open(file, 'r') as filehandle:
                FileHandle(filehandle)
        except OSError as err:
            print("ERROR: can't open '{}' for read: {} (errno={})".format(err.filename, err.strerror, err.errno),
                  file=sys.stderr)

if __name__ == '__main__':
    if len(sys.argv) >= (1+1):
        main(sys.argv[1])
    else:
        pname = os.path.basename(__file__)
        print('{} {}\n\nUSAGE: {} <name>.py'.format(pname, __version__, pname), file=sys.stderr)

# EOF #
