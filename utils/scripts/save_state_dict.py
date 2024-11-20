# Save the state_dict of a model
import sys
import torch

model_file = sys.argv[1]
state_dict_file = sys.argv[2]

model = torch.load(model_file).to(torch.device('cpu'))
torch.save(model.state_dict(), state_dict_file)
