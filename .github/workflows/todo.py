import github
import argparse
import git
import re
import os


def getTODOS():
    pattern = re.compile('TO' + 'DO:', re.IGNORECASE)
    # Iterate over the files in the current directory and its subdirectories
    res={}
    for dirpath, dirnames, filenames in os.walk('.'):
        # Exclude the .git directory
        if '.git' in dirnames:
            dirnames.remove('.git')
        for filename in filenames:
            # Skip files with non-textual extensions
            if not filename.endswith('.md') and not filename.endswith('.py') and not filename.endswith('.cpp') and not filename.endswith('.h'):
                continue
            filepath = os.path.join(dirpath, filename)
            with open(filepath, 'r') as file:
                for line_number, line in enumerate(file, 1):
                    if pattern.search(line):
                        if not res.get(filepath):
                            res[filepath]=[]
                        res[filepath].append([line_number, line.lower().split('to' + 'do:')[1]])
    return res


# TODO: add function context?
def genMarkdown(todos):
    res=""
    for fname,file_todos in todos.items():
        res+=f"## {fname}\n"
        for line_num, todo_message in file_todos:
            res+=f"- [ ] Line {line_num}: {todo_message}\n"
        res+="\n"
    return res



parser= argparse.ArgumentParser()

parser.add_argument('--token', action="store", dest="token")
parser.add_argument('--repo', action="store", dest="repo")

args=parser.parse_args()
ghInst=github.Github(args.token)
repo=ghInst.get_repo(args.repo)
issues = repo.get_issues(state='all')
issue_name=f'TODOs:{git.Repo(".").active_branch.name}'
for issue in issues:
    if issue.title == issue_name:
        issue_number = issue.number
        break
else:
    print("Issue not found, creating one")
    issue=repo.create_issue(title=issue_name, body='')
    issue_number=issue.number

markdown=genMarkdown(getTODOS())
if markdown!="":
    repo.get_issue(issue_number).edit(body=markdown, state="open")
else:
    repo.get_issue(issue_number).edit(state="closed")
