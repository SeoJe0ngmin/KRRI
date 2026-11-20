from .runtime import parser,run
if __name__=='__main__':
    run(parser().parse_args())
