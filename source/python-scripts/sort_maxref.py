#!/usr/bin/python3
import os
import re
import xml.etree.ElementTree as ET
from xml.etree.ElementTree import ElementTree

path = "../../docs"
extra_content_path = '../projects'

sort_elements = [
    ['inletlist', 'inlet', 'id'],
    ['outletlist', 'outletl', 'id'],
    ['methodlist', 'method', 'name'],
    ['jittermethodlist', 'jittermethod', 'name'],
    ['attributelist', 'attribute', 'name'],
    ['seealsolist', 'seealso', 'name']
]


def natural_sort(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(l, key=alphanum_key)


def add_ref_page_extra(tree: ElementTree, object_name: str) -> ElementTree:
    # check if we have a corresponding ref_page_extra.xml file
    extras_path = extra_content_path + '/' + object_name + '/ref_page_extra.xml'
    if not os.path.exists(extras_path):
        return tree
    root = tree.getroot()
    extras_tree = ET.parse(extras_path)
    extras_root = extras_tree.getroot()

    # ad discussion section
    extra_discussion = extras_root.find('discussion')
    if extra_discussion is not None:
        discussion = root.find('discussion')
        if discussion is not None:
            root.remove(discussion)
        root.append(extra_discussion)

    # add method arguments section
    extra_method_list = extras_root.find('methodlist')
    extra_methods_args = {}
    if extra_method_list is not None:
        extra_methods = list(extra_method_list.findall('method'))
        for extra_method in extra_methods:
            extra_arg_list = extra_method.find('arglist')
            extra_methods_args[extra_method.attrib['name']] = extra_arg_list
    for extra_meth_name in extra_methods_args:
        extra_meth_arg_list = extra_methods_args[extra_meth_name]
        # look for method name in root
        method_list = root.find('methodlist')
        if method_list is not None:
            methods = method_list.findall('method')
            if methods is not None:
                for method in methods:
                    meth_name = method.attrib['name']
                    if meth_name == extra_meth_name:
                        existing_meth_arg_list = method.find('arglist')
                        if existing_meth_arg_list is not None:
                            method.remove(existing_meth_arg_list)
                        method.append(extra_meth_arg_list)

    # add misc sections
    extra_misc_tags = extras_root.findall('misc')
    if extra_misc_tags is not None:
        misc_tags = root.findall('misc')
        if misc_tags is not None:
            for misc_tag in misc_tags:
                root.remove(misc_tag)
        for extra_misc_tag in extra_misc_tags:
            root.append(extra_misc_tag)
    return tree


def sort_ref_entries(tree: ElementTree, container_type: str = "methodlist", tag_type: str = "method",
                     sort_attr: str = "name") -> ElementTree:
    root = tree.getroot()

    container = root.find(container_type)
    if container is None:
        return tree

    tags = list(container.findall(tag_type))

    by_sort_attr = {}
    sort_keys = []
    for t in tags:
        by_sort_attr[t.attrib[sort_attr]] = t
        sort_keys.append(t.attrib[sort_attr])

    sorted_keys = natural_sort(sort_keys)

    tags.sort(key=lambda el: str(el.attrib[sort_attr]))

    for item in container.findall(tag_type):
        container.remove(item)

    for k in sorted_keys:
        container.append(by_sort_attr[k])
    return tree


def print_file_list(files: list) -> list:
    selected_files = []
    i = 0
    for f in files:
        print(f"[{i}] {f}")
        i = i + 1
    print(f"[all] format all")
    print(f"[x] exit")

    print("Chose file to format:")
    choice = input().lower()
    if choice == 'all':
        selected_files = files
    elif choice == 'x':
        exit(0)
    else:
        try:
            file_index = int(choice)
            if file_index > len(files) - 1:
                raise ValueError()
        except ValueError:
            print("Invalid choice")
            return print_file_list(files)
        selected_files = [files[file_index]]
    return selected_files


def main():
    if not os.path.exists(path):
        print(f"{path} not found.")
        exit(0)
    files = os.listdir(path)
    files = [
        f for f in files
        if os.path.isfile(path + '/' + f) and f.endswith('maxref.xml')
    ]
    if len(files) == 0:
        print(f"No .maxref.xml files found in{path}.")
        exit(0)
    selected_files = print_file_list(files)
    for file in selected_files:
        object_name = file[:-11]
        tree = ET.parse("../../docs/" + file)
        # sort entries alphabetically
        for sort_element in sort_elements:
            tree = sort_ref_entries(tree, sort_element[0], sort_element[1], sort_element[2])

        # add extra
        add_ref_page_extra(tree, object_name)
        out_path = "../../docs/" + file
        tree.write(out_path, encoding='utf-8', xml_declaration=True)
    exit(0)


if __name__ == '__main__':
    main()
