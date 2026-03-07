from client import MessageType, DebugClient
import logging


class Command:
    def __init__(self, name: str, handler: callable, help_message: str,
                 args: list[str] | None = None, client: DebugClient | None = None,
                 msg_type: MessageType | None = None):

        # string you call the command by
        self.name = name

        self.handler = handler
        self.help_message = help_message

        self.args = args

        if (bool(client) or bool(msg_type)) and (not bool(client) and bool(msg_type)):
            logging.warning(f"Please define both client and msg_type for command {
                            name} ITS REALLY IMPORTANT or define neither")

        self.client = client
        self.msg_type = msg_type

    def validate_args(self, args: dict) -> int:
        for k, _ in args.items():
            if k not in self.args:
                return False

        return True

    def call(self, args: dict | None = None):
        if args is not None and not self.validate_args(args):
            logging.error(f"missing arg {self.name}")
            return

        if self.client is None:
            self.handler(args)
            return

        self.client.handle(self.msg_type, args)

    def __str__(self) -> str:
        result = f"{self.name}:\n{self.help_message}\n\n"

        result += f"  usage:  {self.name}"
        for arg in self.args:
            result += " [" + arg + "]"

        result += "\n"

        return result


commands: dict[str, Command] = {}


def regiser_command(name: str, handler: callable, help_message: str,
                    args: list[str] | None = None, client: DebugClient | None = None,
                    msg_type: MessageType | None = None):

    command = Command(name, handler, help_message, args, client, msg_type)

    commands[name] = command

    if client is not None:
        client.register_handler(msg_type, handler)


def print_help_messages(command: list[str]):
    if len(command) > 1:
        print(commands[command[1]].__str__())
        return

    # print general help command
    print("CLI for controlling the drone here are all da commands:")
    for k, v in commands.items():
        print(v.__str__())


def start():
    user_input = ''
    while user_input not in ['q', 'quit', 'exit']:
        user_input = input("Enter a command: ")
        args = user_input.strip().split(' ')
        command = args[0]

        # commands that dont need handlers
        if command in ['q', 'quit', 'exit']:
            break
        elif command in ['h', 'help']:
            print_help_messages(args)
            continue

        command = commands.get(command)

        if command is None:
            print("Command not found")
            continue

        if len(command.args) == 0:
            command.call()
            continue

        argc = len(args)
        command_argc = len(command.args)
        if argc - 1 < command_argc:
            print(f"missing arg, {command.__str__()}")
            continue
        elif argc - 1 > command_argc:
            print(f"Expected {command_argc} args, got {argc}")
            continue

        # start at 1 to ignore command
        command_args_dict = {command.args[i-1]: args[i]
                             for i in range(1, len(args))}
        command.call(command_args_dict)
