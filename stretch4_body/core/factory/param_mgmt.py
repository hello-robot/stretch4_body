import stretch4_body.core.hello_utils as hello_utils
import importlib
import click

def generate_user_params_from_template(model_name, fleet_dir=None):
    param_module_name = 'stretch4_body.robot.robot_params_' + model_name
    user_params = getattr(importlib.import_module(param_module_name), 'user_params_template')
    user_header = getattr(importlib.import_module(param_module_name), 'user_params_header')
    hello_utils.write_fleet_yaml('stretch_user_params.yaml',user_params,fleet_dir,user_header)

def generate_configuration_params_from_template(model_name, batch_name, robot_serial_no, fleet_dir=None):
    param_module_name = 'stretch4_body.robot.robot_params_' + model_name
    config_params = getattr(importlib.import_module(param_module_name), 'configuration_params_template')
    config_header = getattr(importlib.import_module(param_module_name), 'configuration_params_header')
    config_params['robot']['batch_name']=batch_name
    config_params['robot']['serial_no']=robot_serial_no
    hello_utils.write_fleet_yaml('stretch_configuration_params.yaml', config_params, fleet_dir,config_header)

def copy_over_params(dest_dict, src_dict,dest_dict_name='',src_dict_name=''):
    """
    Copy atomic values (list, numbers, strings) from src to dest
    Only if the key is found in the dest dict and types are the same
    """
    for k in dest_dict.keys():
        if k in src_dict:
            if type(src_dict[k])==type(dest_dict[k]):
                if type(src_dict[k])==dict:
                    copy_over_params(dest_dict[k],src_dict[k],dest_dict_name+'.'+str(k), src_dict_name+'.'+str(k))
                else:
                    dest_dict[k]=src_dict[k]
            else:
                click.secho('Migration error. Type mismatch for key %s during copy from %s to %s. From type %s. To type %s'%(k,src_dict_name,dest_dict_name,str(type(src_dict[k])),str(type(dest_dict[k])))
                            , fg="red")
                print('Values Src | Dest: ',src_dict[k],dest_dict[k])
        else:
            click.secho('Migration error. Parameter %s not found during copy from %s to %s'%(k,src_dict_name,dest_dict_name), fg="red")


def param_change_check(new_dict,prior_dict,num_warnings,new_dict_name,prior_dict_name,whitelist=[]):
    for k in new_dict.keys():
        if k in prior_dict:
            if type(new_dict[k])==dict:
                    num_warnings=param_change_check(new_dict[k],prior_dict[k],num_warnings,new_dict_name+'.'+k,prior_dict_name+'.'+k,whitelist)
            else:
                if new_dict[k]!=prior_dict[k]:
                    whitelisted = False
                    whitelist_name = prior_dict_name + '.' + k
                    whitelist_name = whitelist_name[whitelist_name.find('.') + 1:]  # eg robot.batch_name
                    for w in whitelist:
                        if w == whitelist_name:
                            whitelisted = True
                    if not whitelisted:
                        click.secho('Warning. Value change in %s from %s to %s'%(whitelist_name,prior_dict[k],new_dict[k]), fg="yellow")
                        num_warnings=num_warnings+1
    return num_warnings


def param_added_check(new_dict,prior_dict,num_warnings,new_dict_name,prior_dict_name,whitelist=[]):
    for k in new_dict.keys():
        if k in prior_dict:
            if type(new_dict[k])==dict:
                    num_warnings=param_added_check(new_dict[k],prior_dict[k],num_warnings,new_dict_name+'.'+k,prior_dict_name+'.'+k,whitelist)
        else:
            whitelisted = False
            whitelist_name = prior_dict_name+'.'+k
            whitelist_name = whitelist_name[whitelist_name.find('.') + 1:]  # eg robot.batch_name
            for w in whitelist:
                if w == whitelist_name:
                    whitelisted = True
            if not whitelisted:
                click.secho('Warning. Parameter introduced: %s'%whitelist_name, fg="yellow")
                num_warnings=num_warnings+1
    return num_warnings

def param_dropped_check(new_dict,prior_dict,num_warnings,new_dict_name,prior_dict_name,whitelist=[]):
    for k in prior_dict.keys():
        if k in new_dict:
            if type(prior_dict[k])==dict:
                    num_warnings=param_dropped_check(new_dict[k],prior_dict[k],num_warnings,new_dict_name+'.'+k,prior_dict_name+'.'+k,whitelist)
        else:
            whitelisted=False
            whitelist_name = prior_dict_name+'.'+k
            whitelist_name=whitelist_name[whitelist_name.find('.')+1:] #eg robot.batch_name
            for w in whitelist:
                if w==whitelist_name:
                    whitelisted=True
            if not whitelisted:
                click.secho('Warning. Parameter %s dropped'%whitelist_name, fg="yellow")
                num_warnings=num_warnings+1
    return num_warnings

