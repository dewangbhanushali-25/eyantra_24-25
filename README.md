# eYRC_3152
Contains all folders and files used for EYRC competition. 

made by love by DJ Sanghvi Ke Chode

# Git commands
## general
> to intialise git in folder
```command
 git init
```
> adds all the files to stage
```command
 git add .
```
> clones the directary present on github
```command
 git clone <url> 
```
> commits directory with messange for understanding
```command
git commit -m "message"
```
>  push the origin branch already having itss upstream branch ready
```command
git push
```
>  to push the required branch to remote also setting its upstream branch
```command
 git push --set-upstream origin <brach_name>
```
>  to view all the branches setup locally
```command
 git branch
```
>  to create and switch new branch in one command
```command
git checkout -b <branch_name>
```
>  to switch to particular branch locally
```command
git switch <branch_name >
```
>  to generate new tag (first checkout to main)
```command
git tag <tagname>
```
>  to push a particular tag
```command
git push <repo_url> <tag name>
```
## To pull all directories excluding project envelope follow this
>  do not init if already done
```command
 git init
```
```command
git remote add origin <url>
```
```command
git pull origin master 
```
