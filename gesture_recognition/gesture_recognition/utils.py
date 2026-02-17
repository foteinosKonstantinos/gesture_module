# import torch
# import torchvision.models
# import torchvision.transforms as transforms

# CLASSES = [
#     "fetch-a-gas-mask",
#     "come-to-me",
#     "ok-to-go",
#     "move-away-from-here",
#     "operation-finished",
#     "freeze",
#     "emergency-situation",
#     "i-need-help",
#     "evacuate-the-area",
#     "i-lost-connection",
#     "fetch-a-shovel",
#     "fetch-an-axe"
# ]

# class HGRModelWrapper:

#     def __init__(self, classifier:torch.nn.Module, device:str="cpu", classes:list[str]=CLASSES):
#         self.__classifier = classifier
#         self.__classifier.eval()
#         self.__classes = classes
#         self.__transforms = transforms.Compose([transforms.ToTensor(), transforms.Resize((224,224))])
#         self.__device = device

#     def to(self, device:str) -> None:
#         self.__classifier = self.__classifier.to(device)
#         self.__device = device

#     def from_pil(self, pil_image:str) -> torch.Tensor:
#         return self.__transforms(pil_image).unsqueeze(dim=0)
#         # Image.open(path) publisher

#     @torch.no_grad()
#     def predict_from_tensor3d(self, tensor3d):
#         logits = self.__classifier(tensor3d.to(self.__device))[0]
#         probabilities = torch.nn.functional.softmax(logits)
#         return {
#             "class": self.__classes[logits.argmax()],
#             "probabilities": {
#                 self.__classes[i]:probabilities[i].item() for i in range(len(self.__classes))
#             }
#         }

#     @torch.no_grad()
#     def predict_from_pil(self, pil_image):
#         return self.predict_from_tensor3d(tensor3d=self.from_pil(pil_image))
    
# def convert_json_to_tensor()