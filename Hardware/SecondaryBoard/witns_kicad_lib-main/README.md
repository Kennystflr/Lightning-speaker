# WITNS Kicad Library

## Add to your project
```bash
git submodule add git@github.com:lfiack/witns_kicad_lib.git
```

## Don't forget to document your project
Add the following lines to your README.md :
````
## Make the project work
This project uses a submodule for the KiCAD library. To get it properly, use the `--recursive` option when cloning the repo :

```bash
git clone --recursive git@github.com:<path_to_your_project>
```

If you forgot the `--recursive` option (as I do quite often), you can type the following command :

```bash
git submodule update --init --recursive
```
````
