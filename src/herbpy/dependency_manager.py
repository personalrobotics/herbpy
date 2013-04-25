import herbpy, rospkg, os

def append_to_env(variable, new_path, separator=':'):
    paths = os.environ.get(variable, '')
    if paths:
        paths_list = paths.split(separator)
    else:
        paths_list = list()

    # Check if we're adding a duplicate entry.
    duplicate = False
    new_canonical_path = os.path.realpath(new_path)
    for path in paths_list: 
        canonical_path = os.path.realpath(path)
        if canonical_path == new_canonical_path:
            duplicate = True
            break

    # Add the new path to the environmental variable.
    if not duplicate:
        paths_list.insert(0, new_path)
        paths = separator.join(paths_list)
        os.environ[variable] = paths

    return paths 

def export_paths(package_name):
    pkg = rospkg.RosPack()
    try:
        dependencies = pkg.get_depends(package_name)
    except rospkg.ResourceNotFound:
        return False

    plugin_paths = list()
    data_paths = list()
    for package in dependencies:
        # Load the package manifest.
        try:
            manifest = pkg.get_manifest(package_name)
        except rospkg.ResourceNotFound:
            herbpy.logger.warning('Dependency %s was not found.', package_name)
            return False

        # Process the OpenRAVE export tags. 
        for plugin_path in manifest.get_export('openrave', 'plugins'):
            herbpy.logger.info('Adding %s to OPENRAVE_PLUGINS.', plugin_path)
            plugin_paths = append_to_env('OPENRAVE_PLUGINS', plugin_path)

        for data_path in manifest.get_export('openrave', 'data'):
            herbpy.logger.info('Adding %s to OPENRAVE_DATA.', data_path)
            data_paths = append_to_env('OPENRAVE_DATA', data_path)

    return True
