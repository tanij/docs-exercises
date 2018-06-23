import os
import sys

from contracts.utils import check_isinstance  # @UnresolvedImport
from duckietown_utils import logger
from duckietown_utils.friendly_path_imp import friendly_path
from duckietown_utils.image_writing import write_image_as_jpg
from duckietown_utils.instantiate_utils import indent
from duckietown_utils.jpg import image_cv_from_jpg
from duckietown_utils.system_cmd_imp import system_cmd_result, contract
import numpy as np


@contract(retcode=int, cmd=str, args=list)
def expect(retcode, cmd, args, desc=None, our_exit=255):
    check_isinstance(cmd, str)
    check_isinstance(args, list)
    cwd = '.'
    cmd0 = [cmd]
    cmd0.extend(args)
    
    cmd_friendly = [friendly_path(cmd)] + list(args)
    
    logger.info('cmd: %s' % cmd_friendly)
    try:
        res = system_cmd_result(cwd, cmd0,
                          display_stdout=False,
                          display_stderr=False,
                          raise_on_error=False)
    except OSError:
        raise
    
    if res.ret != retcode:
        msg = ""
        if desc is not None:
            msg += desc
        msg += '\nExpected retcode %d, got %d.' % (retcode, res.ret)
        msg += '\nCommand:\n  ' + " ".join(cmd0) 
        msg += '\n' + indent(res.stdout, '|', ' stdout |')
        msg += '\n' + indent(res.stderr, '|', ' stderr |')
        logger.error(msg)
        logger.error('Exiting with code %d' % our_exit)
        sys.exit(our_exit)

# def mkdirs_thread_safe(dst):
#     """ Make directories leading to 'dst' if they don't exist yet"""
#     if dst == '' or os.path.exists(dst):
#         return
#     head, _ = os.path.split(dst)
#     if os.sep == ':' and not ':' in head:
#         head += ':'
#     mkdirs_thread_safe(head)
#     try:
#         mode = 511  # 0777 in octal
#         os.mkdir(dst, mode)
#     except OSError as err:
#         if err.errno != 17:  # file exists
#             raise
# 
# 
# def make_sure_dir_exists(filename):
#     """ Makes sure that the path to file exists, but creating directories. """
#     dirname = os.path.dirname(filename)
#     # dir == '' for current dir
#     if dirname != '' and not os.path.exists(dirname):
#         mkdirs_thread_safe(dirname)
# 
# 
# def write_image_as_jpg(image, filename):
#     make_sure_dir_exists(filename)
#     with open(filename, 'w') as f:
#         f.write('tmp')
        
# from duckietown_utils.image_writing import write_image_as_jpg


def panic(retcode, msg):
    logger.error(msg)
    sys.exit(retcode)

def image_ops_generic_version(forgot_arg_check=False, 
                              forgot_filename_check=False, 
                              forgot_check_valid=False):

    args = sys.argv[1:]

    if not forgot_arg_check:
        if len(args) != 2:
            panic(99, 'Argument not given')

    filename = args[0]
    dirname = args[1]

    if not forgot_filename_check:

        if not os.path.exists(filename):
            panic(2, 'Filename does not exist: %s' % filename)

    data = open(filename).read()
    
    try:
        image = image_cv_from_jpg(data)
    except ValueError as e:
        if forgot_check_valid:
            raise
        else:
            panic(3, 'Cannot decode JPG: %s' % e)
            
    
    image_flip = np.flipud(image)
    side = np.hstack((image, image_flip))
    
    write_image_as_jpg(image, os.path.join(dirname, 'original.jpg'))
    write_image_as_jpg(image_flip, os.path.join(dirname, 'flip.jpg'))
    write_image_as_jpg(side, os.path.join(dirname, 'side.jpg'))
    

