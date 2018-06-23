#!/usr/bin/env python

from image_ops_generic import *
 
     
def image_ops_tester_generic(
        forgot_to_check_empty=False, 
         forgot_to_check_truncated=False,
         forgot_to_check_not_existing=False,
         forgot_to_check_garbage=False
             ):
    
    args = sys.argv[1:]
    
    if len(args) != 1:
        panic(1, 'Exactly one parameter required.')
        
    script = args[0]
    
    script = os.path.realpath(script)
    
    if not forgot_to_check_empty:
        expect(99, script, [], desc='Empty arguments should return 99.',
               our_exit=11)
    
    good_images = ['sunset.jpg']
    where = '/tmp/where'
    for good_image in good_images:
        expect(0, script, [good_image, where], our_exit=11)

    
    if not forgot_to_check_not_existing:
        not_existing = 'not-existing.jpg'
        
        expect(2, script, [not_existing, where], our_exit=11)
    
    invalid = []
    
    
    if not forgot_to_check_empty:
        invalid_empty = where + '/empty.jpg'
        with open(invalid_empty, 'w') as f:
            f.write('')
        invalid.append(invalid_empty)
        
    if not forgot_to_check_garbage:
        invalid_garbage = where + '/garbage.jpg'
        with open(invalid_garbage, 'w') as f:
            f.write('garbage')
        invalid.append(invalid_garbage)

    if not forgot_to_check_truncated:
        invalid_truncated = where + '/truncated.jpg'
        data = open(good_images[0]).read()
        data = data[:-200] 
        with open(invalid_truncated, 'w') as f:
            f.write(data)
        invalid.append(invalid_truncated)
    
    for i in invalid:
        assert os.path.exists(i)
        expect(3, script, [i, where], our_exit=11)
        
    logger.info('image-ops-tester thinks %r is image-ops conformant ' % script)
    
