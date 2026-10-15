from transformers import AutoTokenizer, BertModel, BertTokenizer, RobertaModel, RobertaTokenizerFast
import os
from pathlib import Path

GROUNDINGDINO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BERT_PATH = GROUNDINGDINO_ROOT / "bert-base-uncased"
 
def get_tokenlizer(text_encoder_type):
    # import ipdb;ipdb.set_trace();
    if not isinstance(text_encoder_type, str):
        # print("text_encoder_type is not a str")
        if hasattr(text_encoder_type, "text_encoder_type"):
            text_encoder_type = text_encoder_type.text_encoder_type
        elif text_encoder_type.get("text_encoder_type", False):
            text_encoder_type = text_encoder_type.get("text_encoder_type")
        elif os.path.isdir(text_encoder_type) and os.path.exists(text_encoder_type):
            pass
        else:
            raise ValueError(
                "Unknown type of text_encoder_type: {}".format(type(text_encoder_type))
            )
    print("final text_encoder_type: {}".format(text_encoder_type))
    
    # 新添加代码片段
    tokenizer_path = DEFAULT_BERT_PATH
    tokenizer = BertTokenizer.from_pretrained(str(tokenizer_path), use_fast=False)
    return tokenizer
 
    '''
    tokenizer = AutoTokenizer.from_pretrained(text_encoder_type)
    return tokenizer
    '''
    
 
def get_pretrained_language_model(text_encoder_type):
    # import ipdb;ipdb.set_trace();
    if text_encoder_type == "bert-base-uncased" or (os.path.isdir(text_encoder_type) and os.path.exists(text_encoder_type)):
        # 新添加代码片段
        model_path = DEFAULT_BERT_PATH
        return BertModel.from_pretrained(str(model_path))
        # return BertModel.from_pretrained(text_encoder_type)
    if text_encoder_type == "roberta-base":
        return RobertaModel.from_pretrained(text_encoder_type)
 
    raise ValueError("Unknown text_encoder_type {}".format(text_encoder_type))
