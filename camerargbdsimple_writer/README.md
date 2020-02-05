```
```
#
``` camerargbdsimple_writer
```
Intro to component here


## Configuration parameters
As any other component,
``` *camerargbdsimple_writer* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    EXAMPLE HERE

    
## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <camerargbdsimple_writer 's path> ```

    cp etc/config config
    
After editing the new config file we can run the component:

    bin/

```camerargbdsimple_writer ```

    --Ice.Config=config
