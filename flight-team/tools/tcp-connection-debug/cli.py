from client import MessageType, DebugClient
import logging


class Command:
    def __init__(self, name: str, handler: callable, help_message: str,
                 args: list[str] = None, client: DebugClient = None,
                 msg_type: MessageType = None):

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

    def validate_args(self, args: dict) -> bool:
        for k, v in args.items():
            if k not in self.args:
                return False

        return True

    def call(self, args: dict = None):
        if args is not None and not self.validate_args(args):
            logging.error(f"you registered an arg in {self.name} incorrectly")

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
                    args: list[str] = None, client: DebugClient = None,
                    msg_type: MessageType = None):

    command = Command(name, handler, help_message, args, client, msg_type)

    commands[name] = command

    if client is not None:
        client.register_handler(msg_type, handler)


def print_help_messages(command: list[str]):
    if command is not None:
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
        if len(args) <= 1:
            args = None

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

        if args is None:
            command.call()
            continue

        # start at 1 to ignore command
        command_args_dict = {command.args[i-1]: args[i]
                             for i in range(1, len(args))}
        command.call(command_args_dict)
